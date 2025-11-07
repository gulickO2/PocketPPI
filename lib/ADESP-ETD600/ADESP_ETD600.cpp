#include <HardwareSerial.h>
#include "ADESP_UI.h"
#include "ADESP_Aircraft.h"
#include "ADESP_ETD600.h"

//#######################################################################
// *GLOBALS
//#######################################################################
extern ADESP_HX8357 tft;
extern ADESP_STMPE610 ts;
ADESP_UI ui;

HardwareSerial ADSB_Serial(2); // UART2 for ADS-B module

SemaphoreHandle_t xMutex;
QueueHandle_t xQueueEventRx, xQueueTouchEvents;

program_states_t statesADESP;
hw_timer_t *timer5s = NULL;

mydata_t my;
std::unordered_map<uint32_t, Aircraft> aircraftList;
TaskHandle_t xHandleInitTask, xHandleMainTask, xHandleEventsTask, xHandleTouchTask, xHandleOverlayTask, xHandleADSBReaderTask;

int Airport::index = 0;

// Forward declarations for internal functions
void processADS_B_Message(const String& message);
void createTestAircraft(uint32_t icao, const String& rawMessage);

//#######################################################################
// *Fx: GET RUNTIME SECONDS
//#######################################################################
static int seconds() {
	return round(millis()/1000);
}

//#######################################################################
// *TASK >>> ADS-B DATA READER
//#######################################################################
static void TASK_ADS_B_READER(void *pvParameters) {
    String messageBuffer = "";
    
    for(;;) {
        while (ADSB_Serial.available()) {
            char c = ADSB_Serial.read();
            
            if (c == '*') {
                messageBuffer = "*";
            } else if (c == ';' && messageBuffer.length() > 0) {
                messageBuffer += ';';
                processADS_B_Message(messageBuffer);
                messageBuffer = "";
            } else if (messageBuffer.length() > 0 && messageBuffer.length() < 100) {
                messageBuffer += c;
            }
            
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void processADS_B_Message(const String& message) {
    if (message.length() >= 8) {
        String icaoHex = message.substring(1, 7);
        uint32_t icao = strtoul(icaoHex.c_str(), NULL, 16);
        createTestAircraft(icao, message);
    }
}

void createTestAircraft(uint32_t icao, const String& rawMessage) {
    dataFrame_t frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.header.type = ADSB_LOCATION;
    frame.header.id.aircraft = icao;
    
    // Test data - replace with real ADS-B decoding later
    static int testOffset = 0;
    frame.location.lat = 40.0 + (sin(testOffset * 0.1) * 0.5);
    frame.location.lon = -75.0 + (cos(testOffset * 0.1) * 0.5);
    frame.location.alt = 15000 + (testOffset % 5000);
    
    testOffset++;
    
    LOG_INFO("ADS-B DATA | RAW: %s", rawMessage.c_str());
    LOG_1VAR("ICAO", icao);
    
    ADESP_PROCESS_FRAME(frame);
}

//#######################################################################
// *TASK >>> TFT/UI INIT TASK
//#######################################################################
static void TASK_INIT(void *pvParameters) {
	xSemaphoreTake( xMutex, portMAX_DELAY);

	// Init SPI Flash FS
	if(!SPIFFS.begin(true)) LOG_ERROR("INIT TASK | SPIFFS MOUNT FAILED");

	// Init SD Card FS
  	if(!SD.begin(14, SPI, 40000000)) LOG_ERROR("INIT TASK | SDCARD MOUNT FAILED");

	// Initialize TFT LCD & UI
	ui.init();
	LOG_INFO("INIT TASK | UI INITIALIZED");	
	
	// Initialize ADS-B UART (921600 baud for GNS5892R)
	ADSB_Serial.begin(921600, SERIAL_8N1, 16, 17);
	LOG_INFO("INIT TASK | ADS-B UART STARTED AT 921600 BAUD");
	
	// Enable Builtin LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Enable 5s timer interrupt
	timer5s = timerBegin(0, 80, true);
	timerAttachInterrupt(timer5s, &on5sTimer, true);
	timerAlarmWrite(timer5s, 5000000, true);
	timerAlarmEnable(timer5s);

	LOG_INFO("INIT TASK | INIT COMPLETE!");	
	xSemaphoreGive( xMutex );
	vTaskDelete(NULL); 
}

//#######################################################################
// *TASK >>> DRAW OVERLAY
//#######################################################################
static void TASKS_OVERLAY(void *pvParameters) {
	for(;;) {
		ui.drawOverlay();
		vTaskDelay(OVERLAY_DRAW_DELAY / portTICK_PERIOD_MS);
	}
}

//#######################################################################
// *TASK >>> TOUCH EVENTS
//#######################################################################
static void TOUCH_EVENTS(void *pvParameters) {
	for(;;) {
		if( ts.ISR_Flag() ) {
			uint8_t zR;
			uint16_t xR = 0, yR = 0;
			uint16_t tsX = 0, tsY = 0;

			while( ts.bufferSize() > 0 ) {
				ts.readData(&xR, &yR, &zR);
				if( ts.bufferSize() == 1 ) {
					tsX = map(yR, TS_MINX, TS_MAXX, 0, TFT_WIDTH2 );
					tsY = map(xR, TS_MINY, TS_MAXY, 0, TFT_HEIGHT2);
				}
			}

			ts.interruptReset();
			
			if(tsX > 0 && tsY > 0) {			
				uint8_t buttonID = ui.processInput( {tsX, tsY} );
				
				if(statesADESP.debugTouchInput) {
					tft.drawCircle(tsX, tsY, 2, RED);
					Serial.printf("%i - %i | %i - %i", tsX, yR, tsY, xR);					
				}				

				if(buttonID == 0) {}
				else if(buttonID == 1) {
					if( ui.buttons[buttonID].isActive() )
						ui.drawAirports();
					else
						ui.removeAirports();
				}
				else if(buttonID == 2) {
					if( ui.buttons[buttonID].isActive() ) tft.setBRT(LCD_DIMMED);
					else tft.setBRT(LCD_ON);
				}
				else if(buttonID == 3) ui.setRange(statesADESP.currentRange - 10);
				else if(buttonID == 4) ui.setRange(statesADESP.currentRange + 10);
				else if(buttonID == 5) ui.setActiveUI(MAIN_UI);
				else if(buttonID == 6) ui.setActiveUI(TRAFFIC_UI);
				else if(buttonID == 7) ui.setActiveUI(WEATHER_UI);
				else if(buttonID == 8) ui.setActiveUI(MAP_UI);
				else if(buttonID == 9) ui.setActiveUI(SETTINGS_UI);
				else if(buttonID == 10) {
					if( ui.buttons[buttonID].isActive() ) tft.setBRT(LCD_DIMMED);
					else tft.setBRT(LCD_ON);
				}
				else if(buttonID == 11) {
					if( ui.buttons[buttonID].isActive() ) ui.setBluetoothIcon(true);
					else ui.clearBluetoothIcon(true);
				}
				else if(buttonID == 12) {
					if( ui.buttons[buttonID].isActive() ) statesADESP.debugTouchInput = true;
					else statesADESP.debugTouchInput = false;
				}
			}
		}		
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

//#######################################################################
// *TASK* >>> EVENT QUEUE RX
//#######################################################################
static void TASK_EVENTS(void *pvParameters) {
	LOG_INFO("EVENT TASK | EVENT HANDLER TASK STARTED");	
	
	for(;;) {
		if(xQueueEventRx != 0) {
			eventData_t event;
			xQueueReceive( xQueueEventRx, &(event), portMAX_DELAY);
			
			switch(event.type){
				case ADSB_VECTOR_EVT: {
					LOG_INFO("EVENT TASK | AIRCRAFT VECTOR MSG");
					ui.addAircraftByVector(event.id.aircraft, event.data.vector, ADSB);
					break;
				}
				case ADSB_LOCATION_EVT: {
					LOG_INFO("EVENT TASK | AIRCRAFT LOCATION MSG");
					ui.addAircraftByLocation(event.id.aircraft, event.data.location, ADSB);
					break;
				}
				case REMOVE_STALE_AIRCRAFT: {
					ui.measureBattery();					
					ui.drawBattery(true);
					ui.removeStaleAircrafts();
					break;
				}
				case AIRPORT_LOCATION_EVT: {
					LOG_INFO("EVENT TASK | AIRPORT MSG")
					ui.addAirport(Airport::index++, 
								  Airport(event.id.airport, event.data.location));					
					break;
				}
				default: {
					LOG_ERROR("EVENT TASK | UNKOWN EVENT");
					break;
				}
			}
		}
		taskYIELD();
	}
}

//#######################################################################
// *Fx >>> ADESP SYSTEM INIT
//#######################################################################
void ADESP_SYS_INIT(void) {
	// Startup Messages
	Serial.begin(ADESP_DEBUG_BAUD);	SYS_MSG();
	SYS_MSG("####################################################");
	SYS_MSG(">>                 ADESP-ETD 600                  <<");
	SYS_MSG("####################################################");	
	SYS_MSG("********  WARNING: USE AT YOUR OWN RISK!  **********");
	SYS_MSG("********      EXPERIMENTAL USE ONLY!      **********");
	SYS_MSG("####################################################");	
	SYS_MSGL(">> ADESP-ETD BUILD NUMBER :: ", ADESP_BUILD_NUM);
	SYS_MSGL(">> ADESP-ETD FIRMWARE VERSION :: ", ADESP_VER);
	SYS_MSG(">> INITIALIZING...");

	// Setup mutex and event queue
	xMutex = xSemaphoreCreateMutex();
	xQueueEventRx = xQueueCreate( QUEUE_LENGTH, sizeof(eventData_t) );
	xQueueTouchEvents = xQueueCreate( 20 , sizeof(eventData_t) );
	
	// Load Initial States
	statesADESP.display = ON;
	statesADESP.brightness = LCD_ON;
	statesADESP.currentRange = DEFAULT_RANGE;
	statesADESP.initComplete = false;
	statesADESP.tracking = 0;
	
	statesADESP.MAG = INACTIVE;
	statesADESP.GPS = INACTIVE;
	statesADESP.BLUETOOTH = INACTIVE;

	// Run Startup task
	xTaskCreate(TASK_INIT,
				"INIT", 
				INIT_TASK_STACK_SIZE, 
				NULL, 
				INIT_TASK_PRIORITY, 
				&xHandleInitTask);

	delay(100); 
}

//#######################################################################
// *Fx >>> ADESP SYSTEM START
//#######################################################################
void ADESP_SYS_START(void) {
	xSemaphoreTake( xMutex, portMAX_DELAY);
	SYS_MSG(">> STARTING SYSTEM TASKS...");

	// Event handling task
	SYS_MSG2("   *EVENT SYSTEM TASK... ");
	xTaskCreatePinnedToCore(TASK_EVENTS,
							"EVENT_TSK",
							EVENT_TASK_STACK_SIZE,
							NULL,
							EVENT_TASK_PRIORITY, 
							&xHandleEventsTask, 0);
	SYS_MSG("OK");
	
	// Touch Event handling task
	SYS_MSG2("   *TOUCH SCREEN SYSTEM TASK... ");
	xTaskCreatePinnedToCore(TOUCH_EVENTS,
							"TOUCH_TSK",
							TOUCH_TASK_STACK_SIZE,
							NULL,
							TOUCH_TASK_PRIORITY, 
							&xHandleTouchTask, 0);
	SYS_MSG("OK");	
	
	// Overlay drawing task
	SYS_MSG2("   *SCREEN OVERLAY SYSTEM TASK... ");
	xTaskCreatePinnedToCore(TASKS_OVERLAY,
							"OVERLAY_TSK",
							OVERLAY_TASK_STACK_SIZE,
							NULL,
							OVERLAY_TASK_PRIORITY,
							&xHandleOverlayTask, 1);
	SYS_MSG("OK");

	// ADS-B Reader Task (NEW)
	SYS_MSG2("   *ADS-B READER TASK... ");
	xTaskCreatePinnedToCore(TASK_ADS_B_READER,
							"ADS_B_TSK",
							4096,
							NULL,
							EVENT_TASK_PRIORITY,
							&xHandleADSBReaderTask, 0);
	SYS_MSG("OK");

	delay(500);

	// Initialize touch driver
	if( ts.begin() );
	else if ( !ts.begin() ) tft.fillCircle(123, 14, 4, RED);
	
	xSemaphoreGive( xMutex );	
	statesADESP.initComplete = true;	

	// Set initial UI
	ui.setActiveUI(MAIN_UI);

	SYS_MSG("|>>>>>>>>>>>>>>>>>> SYSTEM READY <<<<<<<<<<<<<<<<<<|");
}

//#######################################################################
// *Fx >>> ADESP PROCESS FRAME (CRITICAL - PROCESSES AIRCRAFT DATA)
//#######################################################################
void ADESP_PROCESS_FRAME(dataFrame_t xDTframe) {
	// ADS-B DATA
	if(xDTframe.header.type == ADSB_LOCATION) {
		LOG_INFO("UART DATA | ADSB_LOCATION FRAME RX");
		eventData_t evt;
		evt.type = ADSB_LOCATION_EVT;
		evt.id.aircraft = xDTframe.header.id.aircraft;
		evt.data.location = xDTframe.location;
		xQueueSend( xQueueEventRx, (void *) &evt, 0 );
	}
	
	if(xDTframe.header.type == ADSB_VECTOR) {
		LOG_INFO("UART DATA | ADSB_VECTOR FRAME RX");
		eventData_t evt;
		evt.type = ADSB_VECTOR_EVT;
		evt.id.aircraft = xDTframe.header.id.aircraft;
		evt.data.vector = xDTframe.vector;
		xQueueSend( xQueueEventRx, (void *) &evt, 0 );
	}

	// MLAT DATA
	if(xDTframe.header.type == MLAT_LOCATION) {
		LOG_INFO("UART DATA | MLAT LOCATION FRAME RX");
		eventData_t evt;
		evt.type = MLAT_LOCATION_EVT;
		evt.id.aircraft = xDTframe.header.id.aircraft;
		evt.data.location = xDTframe.location;
		xQueueSend( xQueueEventRx, (void *) &evt, 0 );
	}
	
	if(xDTframe.header.type == MLAT_VECTOR) {
		LOG_INFO("UART DATA | MLAT VECTOR FRAME RX");
		eventData_t evt;
		evt.type = MLAT_VECTOR_EVT;
		evt.id.aircraft = xDTframe.header.id.aircraft;
		evt.data.vector = xDTframe.vector;
		xQueueSend( xQueueEventRx, (void *) &evt, 0 );
	}

	// UAT DATA
	if(xDTframe.header.type == UAT_LOCATION) {
		LOG_INFO("UART DATA | UAT LOCATION FRAME RX");
		eventData_t evt;
		evt.type = TISB_LOCATION_EVT;
		evt.id.aircraft = xDTframe.header.id.aircraft;
		evt.data.location = xDTframe.location;
		xQueueSend( xQueueEventRx, (void *) &evt, 0 );
	}
	
	if(xDTframe.header.type == UAT_VECTOR) {
		LOG_INFO("UART DATA | UAT VECTOR FRAME RX");
		eventData_t evt;
		evt.type = TISB_VECTOR_EVT;
		evt.id.aircraft = xDTframe.header.id.aircraft;
		evt.data.vector = xDTframe.vector;
		xQueueSend( xQueueEventRx, (void *) &evt, 0 );
	}

	// AIRPORT DATA
	if(xDTframe.header.type == AIRPORT_LOCATION) {
		LOG_INFO("UART DATA | AIRPORT_LOCATION FRAME RX");
		eventData_t evt;
		evt.type = AIRPORT_LOCATION_EVT;
		strcpy(evt.id.airport, xDTframe.header.id.airport);		
		evt.id.airport[3] = 0;
		evt.data.location = xDTframe.location;
		xQueueSend( xQueueEventRx, (void *) &evt, 0 );
	}

	// OWNSHIP DATA
	if(xDTframe.header.type == OWNSHIP_LOCATION) {
		LOG_INFO("UART DATA | OWNSHIP LOCATION FRAME RX");
		my.location = xDTframe.location;
		statesADESP.GPS = RECENT;
		statesADESP.lastGPSupdate = seconds();
	}
	
	if(xDTframe.header.type == OWNSHIP_VECTOR) {
		LOG_INFO("UART DATA | OWNSHIP VECTOR FRAME RX");
		my.vector = xDTframe.vector;
		ui.setCompass(-my.vector.heading, true);
		statesADESP.MAG = ACTIVE;
		statesADESP.lastMAGupdate = seconds();
	}
}

//#######################################################################
// *Fx >>> INTERRUPT HANDLING
//#######################################################################
void IRAM_ATTR onTS_TOUCH() {
	if(statesADESP.initComplete) 
		ts.setISRflag(true);
}

static void IRAM_ATTR on5sTimer() {
	eventData_t event;
	event.type = REMOVE_STALE_AIRCRAFT;
	xQueueSendFromISR( xQueueEventRx, (void*) &event, 0U );
}