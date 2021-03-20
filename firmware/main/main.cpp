// Inspired by esp-idf/examples/ethernet/eth2ap/main/ethernet_example_main.c

#include <string.h>
#include <stdlib.h>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_event.h>
#include <esp_private/wifi.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/raw.h>
#include <nvs_flash.h>

// Parameters received at initialization time
static char networkName[64];
static char networkPass[64] = ""; // unprotected by default
static int wirelessChannel;
static int baudRate;
static wifi_interface_t wirelessInterface;
static int staticIpAddress[4], staticGateway[4], staticNetMask[4]; // STA only
static uint8_t wirelessProtocols;

// DOIT ESP32 DEVKIT V1 (default board)
static constexpr gpio_num_t ledPin = GPIO_NUM_2;
static constexpr int ledOffValue = 0;

// ESP32-CAM
//static constexpr gpio_num_t ledPin = GPIO_NUM_33;
//static constexpr int ledOffValue = 1;

static volatile bool ledShouldBlink, ledShouldBeOff = true;

static RingbufHandle_t wifiToSerial, serialToWifi;

static struct raw_pcb *rawIcmp, *rawIgmp, *rawUdp, *rawUdpLite, *rawTcp;

static void ledTask(void *)
{
	while (true)
	{
		if (ledShouldBeOff)
		{
			gpio_set_level(ledPin, ledOffValue); // off
		}
		else if (ledShouldBlink)
		{
			gpio_set_level(ledPin, ledOffValue); // off
			vTaskDelay(20 / portTICK_PERIOD_MS);

			gpio_set_level(ledPin, !ledOffValue); // on
			ledShouldBlink = false;
		}
		else
		{
			gpio_set_level(ledPin, !ledOffValue); // on
		}

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

static void initializeLed()
{
	// Blue LED
	gpio_reset_pin(ledPin);
	gpio_set_direction(ledPin, GPIO_MODE_OUTPUT);

	xTaskCreate(ledTask, "led", 1024, nullptr, tskIDLE_PRIORITY + 3, nullptr);
}

static void wifiTxTask(void *)
{
	size_t len;

	while (true)
	{
		char *buffer = (char*)xRingbufferReceive(serialToWifi, &len, portMAX_DELAY);
		if (wirelessInterface == WIFI_IF_AP)
		{
			esp_wifi_internal_tx(wirelessInterface, buffer, len); // ignore errors
		}
		else
		{
			struct pbuf *p = pbuf_alloc(PBUF_IP, len, PBUF_RAM);
			if (p != nullptr)
			{
				ip_addr_t dest;

				pbuf_take(p, buffer, len);

				if (IP_HDR_GET_VERSION(p->payload) == 6)
				{
					struct ip6_hdr *ip6hdr = (struct ip6_hdr*)p->payload;
					ip_addr_copy_from_ip6_packed(dest, ip6hdr->dest);
				}
				else
				{
					struct ip_hdr *iphdr = (struct ip_hdr*)p->payload;
					ip_addr_copy_from_ip4(dest, iphdr->dest);
				}

				raw_sendto(rawIcmp, p, &dest);
				pbuf_free(p);
			}
		}
		vRingbufferReturnItem(serialToWifi, buffer);
	}
}

// Used for AP only
static esp_err_t wifiRxCallback(void *buffer, uint16_t len, void *eb)
{
	xRingbufferSend(wifiToSerial, buffer, len, 0);
	esp_wifi_internal_free_rx_buffer(eb);
	return ESP_OK;
}

u8_t onRawInput(void *arg, struct raw_pcb *pcb, struct pbuf *p, const ip_addr_t *addr)
{
	void *item;
	if (xRingbufferSendAcquire(wifiToSerial, &item, p->tot_len, 0))
	{
		pbuf_copy_partial(p, item, p->tot_len, 0);
		xRingbufferSendComplete(wifiToSerial, item);
	}
	pbuf_free(p);

	return 1; // stop further processing by lwip
}

static void onWifiEvent(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	static uint8_t clientCounter = 0;

	switch (event_id)
	{
		case WIFI_EVENT_AP_STACONNECTED:
			if (clientCounter++ == 0)
			{
				ledShouldBeOff = false;
				ESP_ERROR_CHECK(esp_wifi_internal_reg_rxcb(wirelessInterface, wifiRxCallback));
			}
			break;
		case WIFI_EVENT_AP_STADISCONNECTED:
			if (--clientCounter == 0)
			{
				ledShouldBeOff = true;
				ESP_ERROR_CHECK(esp_wifi_internal_reg_rxcb(wirelessInterface, nullptr));
			}
			break;
		case WIFI_EVENT_STA_START:
			rawIcmp = raw_new(IP_PROTO_ICMP);
			rawIgmp = raw_new(IP_PROTO_IGMP);
			rawUdp = raw_new(IP_PROTO_UDP);
			rawUdpLite = raw_new(IP_PROTO_UDPLITE);
			rawTcp = raw_new(IP_PROTO_TCP);

			raw_set_flags(rawIcmp, RAW_FLAGS_HDRINCL);
			raw_set_flags(rawIgmp, RAW_FLAGS_HDRINCL);
			raw_set_flags(rawUdp, RAW_FLAGS_HDRINCL);
			raw_set_flags(rawUdpLite, RAW_FLAGS_HDRINCL);
			raw_set_flags(rawTcp, RAW_FLAGS_HDRINCL);

			raw_recv(rawIcmp, onRawInput, nullptr);
			raw_recv(rawIgmp, onRawInput, nullptr);
			raw_recv(rawUdp, onRawInput, nullptr);
			raw_recv(rawUdpLite, onRawInput, nullptr);
			raw_recv(rawTcp, onRawInput, nullptr);

			esp_wifi_connect();
			break;
		case WIFI_EVENT_STA_CONNECTED:
			ledShouldBeOff = false;
			break;
		case WIFI_EVENT_STA_DISCONNECTED: // connection lost or AP not found during scan
			ledShouldBeOff = true;
			esp_wifi_connect(); // try to reconnect
			break;
		default:
			break;
	}
}

static void initializeWifi(void)
{
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, onWifiEvent, nullptr));

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

	if (wirelessInterface == WIFI_IF_AP)
	{
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

		wifi_config_t wifi_config;
		strcpy((char*)wifi_config.ap.ssid, networkName);
		strcpy((char*)wifi_config.ap.password, networkPass);
		wifi_config.ap.ssid_len = strlen(networkName);
		wifi_config.ap.channel = wirelessChannel;
		if (strlen(networkPass) == 0)
			wifi_config.ap.authmode = WIFI_AUTH_OPEN;
		else
			wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
		wifi_config.ap.ssid_hidden = 0;
		wifi_config.ap.max_connection = 4;
		wifi_config.ap.beacon_interval = 100;
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
	}
	else
	{
		ESP_ERROR_CHECK(esp_netif_init());
		esp_netif_t *netif_sta = esp_netif_create_default_wifi_sta();
		assert(netif_sta != nullptr);

		esp_netif_ip_info_t ip_info;
		ip_info.ip.addr = ESP_IP4TOADDR(staticIpAddress[0], staticIpAddress[1], staticIpAddress[2], staticIpAddress[3]);
		ip_info.gw.addr = ESP_IP4TOADDR(staticGateway[0], staticGateway[1], staticGateway[2], staticGateway[3]);
		ip_info.netmask.addr = ESP_IP4TOADDR(staticNetMask[0], staticNetMask[1], staticNetMask[2], staticNetMask[3]);
		esp_netif_dhcpc_stop(netif_sta);
		esp_netif_set_ip_info(netif_sta, &ip_info);

		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

		wifi_config_t wifi_config;
		strcpy((char*)wifi_config.sta.ssid, networkName);
		strcpy((char*)wifi_config.sta.password, networkPass);
		wifi_config.sta.scan_method = WIFI_FAST_SCAN;
		wifi_config.sta.bssid_set = false;
		wifi_config.sta.channel = wirelessChannel;
		wifi_config.sta.listen_interval = 0;
		wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
		wifi_config.sta.threshold.rssi = 0;
		wifi_config.sta.pmf_cfg.capable = true;
		wifi_config.sta.pmf_cfg.required = false;
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	}

	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
	ESP_ERROR_CHECK(esp_wifi_set_protocol(wirelessInterface, wirelessProtocols));

	ESP_ERROR_CHECK(esp_wifi_start());
	xTaskCreate(wifiTxTask, "wifi_tx", 2048, nullptr, tskIDLE_PRIORITY + 2, nullptr);
}

static void uartTxTask(void *)
{
	size_t len;

	while (true)
	{
		char *buffer = (char*)xRingbufferReceive(wifiToSerial, &len, portMAX_DELAY);

		// Build header
		uint8_t header[4];
		header[0] = 0xAA;
		header[2] = len & 0xFF;
		header[3] = (len >> 8) & 0xFF;

		// Compute checksum
		header[1] = -(header[0] + header[2] + header[3]);
		for (size_t i = 0; i < len; i++)
			header[1] -= buffer[i];

		// Transmit
		ledShouldBlink = true;
		uart_write_bytes(UART_NUM_0, (char*)header, 4);
		uart_write_bytes(UART_NUM_0, buffer, len);

		vRingbufferReturnItem(wifiToSerial, buffer);
	}
}

static void uartRxTask(void *)
{
	while (true)
	{
		static uint8_t buffer[2000]; // bigger than MTU

		// Sync and receive header
		do
			uart_read_bytes(UART_NUM_0, &buffer[0], 1, portMAX_DELAY);
		while (buffer[0] != 0xAA);

		uart_read_bytes(UART_NUM_0, &buffer[1], 1, portMAX_DELAY);
		uart_read_bytes(UART_NUM_0, &buffer[2], 1, portMAX_DELAY);
		uart_read_bytes(UART_NUM_0, &buffer[3], 1, portMAX_DELAY);

		uint16_t len = buffer[2] | (buffer[3] << 8);
		if (4 + len > sizeof(buffer))
			continue; // packet too big

		for (size_t i = 0; i != len; i++)
			uart_read_bytes(UART_NUM_0, &buffer[4 + i], 1, portMAX_DELAY);

		uint8_t acc = 0;
		for (size_t i = 0; i != 4 + len; i++)
			acc += buffer[i];

		if (acc == 0) // good checksum
		{
			ledShouldBlink = true;
			xRingbufferSend(serialToWifi, &buffer[4], len, 0);
		}
	}
}

static void initializeUart()
{
	uart_config_t uartConfig;
	uartConfig.baud_rate = baudRate;
	uartConfig.data_bits = UART_DATA_8_BITS;
	uartConfig.parity = UART_PARITY_DISABLE;
	uartConfig.stop_bits = UART_STOP_BITS_2;
	uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	uartConfig.rx_flow_ctrl_thresh = 0;
	uartConfig.source_clk = UART_SCLK_APB;
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 256, 0, nullptr, 0));
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uartConfig));

	xTaskCreate(uartRxTask, "uart_rx", 2048, nullptr, tskIDLE_PRIORITY + 2, nullptr);
	xTaskCreate(uartTxTask, "uart_tx", 2048, nullptr, tskIDLE_PRIORITY + 2, nullptr);
}

static char *readLine()
{
		static char line[256];
		size_t pos = 0;

		while (1)
		{
			char ch = getchar();
			if (ch == 255) // non-blocking response
			{
				vTaskDelay(1);
				continue;
			}
			else if (ch == '\n')
			{
				break;
			}

			line[pos++] = ch;
		}

		line[pos] = '\0';

		return line;
}

extern "C" void app_main(void)
{
	initializeLed();

	// Ring buffers
	wifiToSerial = xRingbufferCreate(1024 * 32, RINGBUF_TYPE_NOSPLIT);
	serialToWifi = xRingbufferCreate(1024 * 16, RINGBUF_TYPE_NOSPLIT);

	// Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	printf("\n\nready to receive configuration\n\n");

	while (1)
	{
		const char *line = readLine();
		printf("input line: %s\n", line);

		char tmp[16];
		if (sscanf(line, "ssid%s", networkName) == 1)
		{
			printf("got ssid %s\n", networkName);
		}
		else if (sscanf(line, "password%s", networkPass) == 1)
		{
			printf("got password %s\n", networkPass);
		}
		else if (sscanf(line, "channel%d", &wirelessChannel) == 1)
		{
			printf("got channel %d\n", wirelessChannel);
		}
		else if (sscanf(line, "protocols%s", tmp) == 1)
		{
			wirelessProtocols = 0;

			if (strchr(tmp, 'b') != nullptr)
				wirelessProtocols |= WIFI_PROTOCOL_11B;
			if (strchr(tmp, 'g') != nullptr)
				wirelessProtocols |= WIFI_PROTOCOL_11G;
			if (strchr(tmp, 'n') != nullptr)
				wirelessProtocols |= WIFI_PROTOCOL_11N;
			if (strchr(tmp, 'l') != nullptr)
				wirelessProtocols |= WIFI_PROTOCOL_LR;

			printf("got protocols %d\n", wirelessProtocols);
		}
		else if (sscanf(line, "ap%d", &baudRate) == 1)
		{
			printf("got ap with baudrate %d\n", baudRate);
			wirelessInterface = WIFI_IF_AP;
			break;
		}
		else if (sscanf(line, "sta %d.%d.%d.%d %d.%d.%d.%d %d.%d.%d.%d %d", &staticIpAddress[0], &staticIpAddress[1], &staticIpAddress[2], &staticIpAddress[3], &staticGateway[0], &staticGateway[1], &staticGateway[2], &staticGateway[3], &staticNetMask[0], &staticNetMask[1], &staticNetMask[2], &staticNetMask[3], &baudRate) == 13)
		{
			printf("got sta with ip %d.%d.%d.%d gw %d.%d.%d.%d netmask %d.%d.%d.%d baudrate %d\n",
				staticIpAddress[0], staticIpAddress[1], staticIpAddress[2], staticIpAddress[3],
				staticGateway[0], staticGateway[1], staticGateway[2], staticGateway[3],
				staticNetMask[0], staticNetMask[1], staticNetMask[2], staticNetMask[3],
				baudRate);
			wirelessInterface = WIFI_IF_STA;
			break;
		}
	}

	vTaskDelay(500 / portTICK_PERIOD_MS);
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	initializeUart();
	initializeWifi();
}
