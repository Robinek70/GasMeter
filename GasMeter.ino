// Enable debug prints to serial monitor
//#define MY_DEBUG
//#define MY_NODE_ID	20
#define MY_RF24_PA_LEVEL  RF24_PA_HIGH
#define MY_SMART_SLEEP_WAIT_DURATION_MS 250
#define MY_TRANSPORT_WAIT_READY_MS	(2000)
//#define MY_SIGNAL_REPORT_ENABLED
#define MY_SPLASH_SCREEN_DISABLED

// Enable and select radio type attached
#define MY_RADIO_RF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

#define MY_SPECIAL_DEBUG

#include <MySensors.h>
#include <myRPlibs.h>
#include <myRPbattery.h>

#define DOOR_SENSOR_PIN		2 
#define PULSE_SENSOR		3                  // The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)

#define PULSE_FACTOR 100                        // Number of blinks per m3 of your meter (One rotation/liter)

#define SLEEP_MODE true                        // flowvalue can only be reported when sleep mode is false.

#define MAX_FLOW 40                             // Max flow (l/min) value to report. This filters outliers.

#define CHILD_ID		102                             // Id of the sensor child
#define RP_ID_DOOR		80                              // Id of the alarm

#define NODE_VER		F("1.04-3V" STR(MY_RF24_PA_LEVEL)"8")

uint32_t SEND_FREQUENCY =
    60000;           // Minimum time between send (in milliseconds). We don't want to spam the gateway.

MyMessage flowMsg(CHILD_ID,V_FLOW);
MyMessage volumeMsg(CHILD_ID,V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID,V_VAR3);
MyMessage doorMsg(RP_ID_DOOR,V_TRIPPED);


double ppl = ((double)PULSE_FACTOR)/1000;        // Pulses per liter

volatile uint32_t pulseCount = 0;
volatile uint32_t lastBlink = 0;
volatile double flow = 0;
bool pcReceived = false;
uint32_t oldPulseCount = 0;
uint32_t localPulseCount = 0;
uint32_t gwPulseCount = 0;
uint32_t newBlink = 0;
double oldflow = 0;
double volume =0;
double oldvolume =0;
uint32_t lastSend =0;
uint32_t lastPulse =0;
bool wasReport;
byte doorStatus = 0;
byte pulseStatus = 0;
static uint32_t rp_last_force_time = 0;

void before() {
  // initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion)
    pinMode(PULSE_SENSOR, INPUT_PULLUP);
	pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);

	//attachInterrupt(digitalPinToInterrupt(PULSE_SENSOR), onPulse, CHANGE);
	//attachInterrupt(digitalPinToInterrupt(DOOR_SENSOR_PIN), onOpen, CHANGE);

	lastSend = lastPulse = rp_now;
   pulseCount = oldPulseCount = 0;

   /*(new RpBattery(A7, DEFAULT, 500))
		->setBattery(240,500)
		->setDivider(0,1);*/
   (new RpBattery(A7, INTERNAL, 110))
		->setBattery(240,300);
   
   rp_before();
}

void setup()
{
    // Fetch last known pulse count value from gw
    request(CHILD_ID, V_VAR3);
	doorStatus = hwDigitalRead(DOOR_SENSOR_PIN);
    myresend(doorMsg.set(doorStatus));
	pulseStatus = hwDigitalRead(PULSE_SENSOR);

	rp_setup();
}

void presentation()
{
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo(F("Gas Meter"), NODE_VER);

    // Register this device as Gas flow sensor
    present(CHILD_ID, S_GAS, F("Gas Counter"));

	present(RP_ID_DOOR, S_DOOR, F("Gas Door"));  

	present(RP_ID_SIGNAL, S_SOUND);

	present(RP_ID_TEMP, S_TEMP);

	rp_presentation();
}

void loop()
{
	if (!pcReceived)  {
		wait(2000);

		if (!pcReceived) {
			//Last Pulsecount not yet received from controller, request it again
			request(CHILD_ID, V_VAR3);  
			return;
		}			
	}

	rp_loop();

	if(rp_first_loop) {
		reportHwTemp();
	}

	uint32_t currentTime = rp_now;
	
    // Only send values at a maximum frequency or woken up from sleep
    if (SLEEP_MODE || (currentTime - lastSend > SEND_FREQUENCY)) {
        
        if (!SLEEP_MODE && flow != oldflow) {
            oldflow = flow;
#if RP_DEBUG
            Serial.print(F("m3/min:"));
            Serial.println(flow);
#endif
            // Check that we don't get unreasonable large flow value.
            // could happen when long wraps or false interrupt triggered
            if (flow<((uint32_t)MAX_FLOW)) {
                myresend(flowMsg.set(flow, 2));                   // Send flow value to gw
				//wasReport = true;
            }
        }

        // No Pulse count received in 2min
		if(!SLEEP_MODE) {
			if(currentTime - lastPulse > 120000) {
				flow = 0;
			}
		}

        // Pulse count has changed
        if ((pulseCount != oldPulseCount)||(!SLEEP_MODE)) {
            oldPulseCount = pulseCount;
			//wasReport = true;
#ifdef RP_DEBUG
            Serial.print(F("pulsecount:"));
            Serial.println(pulseCount);
#endif
            byte ok = myresend(lastCounterMsg.set(pulseCount/2));                  // Send  pulsecount value to gw in VAR3

            if(ok) {
              gwPulseCount = pulseCount;
            }

            double volume = ((double)(pulseCount/2)/((double)PULSE_FACTOR));
            if ((volume != oldvolume)||(!SLEEP_MODE)) {
                oldvolume = volume;
#ifdef RP_DEBUG
                Serial.print(F("volume:"));
                Serial.println(volume, 3);
#endif
                myresend(volumeMsg.set(volume, 2));               // Send volume value to gw
            }            
        }

		if(((rp_now - rp_last_force_time) > rp_force_time*1000UL*60) || wasReport) {

			rp_signalReport();
			reportHwTemp();

			rp_last_force_time = rp_now;
			//wasReport = true;
		//}
		//if(wasReport && SLEEP_MODE) {
			(void)_sendRoute(build(_msg, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
				I_POST_SLEEP_NOTIFICATION).set(SEND_FREQUENCY));

			(void)_sendRoute(build(_msg, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
				I_PRE_SLEEP_NOTIFICATION).set((uint32_t)MY_SMART_SLEEP_WAIT_DURATION_MS));
			
			wait(MY_SMART_SLEEP_WAIT_DURATION_MS);
			//lastSend = currentTime;			
		}

		wasReport = false;		
    }

	checkPulse();
	checkDoor();
    if (SLEEP_MODE) {
		sleep(digitalPinToInterrupt(DOOR_SENSOR_PIN), CHANGE, digitalPinToInterrupt(PULSE_SENSOR), CHANGE, SEND_FREQUENCY);
		rp_add_sleep_time += SEND_FREQUENCY;

		rp_requreReinit = true;

		checkDoor();
		checkPulse();
		wait(20);

		//flow = 60.*1000.*(pulseCount - oldPulseCount)/SEND_FREQUENCY/PULSE_FACTOR;
        //sleep(digitalPinToInterrupt(PULSE_SENSOR), FALLING, 0 );
        /*Serial.print("wakeup: ");
        Serial.print( millis());
        Serial.print( ", pulseCount: ");
        Serial.println(pulseCount);
		Serial.print( "rp_last_force_time: ");
        Serial.print(rp_last_force_time);
		Serial.print( ", rp_now: ");
		Serial.println(rp_now);
		
		Serial.print( "Temp: ");
		Serial.println(hwCPUTemperature());	*/
    }
	rp_loop_end();
}

void reportHwTemp() {
	//(void)_sendRoute(build(_msg, GATEWAY_ADDRESS, RP_ID_TEMP, C_SET, V_TEMP).set(hwCPUTemperature()));
	static int8_t lastTemp = 0;
	int8_t temp = hwCPUTemperature();
	if(temp!= lastTemp) {
		myresend(build(_msg, GATEWAY_ADDRESS, RP_ID_TEMP, C_SET, V_TEMP).set(temp));
		lastTemp = temp;
	}
}

void receive(const MyMessage &message)
{   
	if (message.isAck()) {
		return;
	}

	rp_receive(message);

	if (message.type==V_VAR3) {
		pulseCount -= gwPulseCount;
		gwPulseCount = 2*message.getULong();
		pulseCount += gwPulseCount;
		flow=oldflow=0;
		rp_addToBuffer("Received from gw:");
		rp_addToBuffer(gwPulseCount/2);
		rp_reportBuffer();
		pcReceived = true;
	}
}

void onPulse()
{
	detachInterrupt(digitalPinToInterrupt(PULSE_SENSOR));
    if (!SLEEP_MODE) {
        uint32_t newBlink = micros();
        uint32_t interval = newBlink-lastBlink;

        if (interval!=0) {
            lastPulse = millis();
            if (interval<500000L) {
                // Sometimes we get interrupt on RISING,  500000 = 0.5 second debounce ( max 120 l/min)
                return;
            }
            flow = (60000000.0 /interval) / ppl;
        }
        lastBlink = newBlink;
    }

	uint32_t enteringMS = 0;
	while (enteringMS < 2000) {
		enteringMS++;
	}

    pulseCount++;
	attachInterrupt(digitalPinToInterrupt(PULSE_SENSOR), onPulse, CHANGE);
	//Serial.println( "count");
}

void checkPulse() {
	byte isClose = hwDigitalRead(PULSE_SENSOR);
	if(pulseStatus != isClose) {
		pulseStatus = isClose;
		pulseCount++;
		//wasReport = true;
	}
}

void checkDoor() {
	byte isOpen = hwDigitalRead(DOOR_SENSOR_PIN);
	if(doorStatus != isOpen) {
		doorStatus = isOpen;
		myresend(doorMsg.set(doorStatus));
		wasReport = true;
	}
}

/*
void onOpen()
{
	detachInterrupt(digitalPinToInterrupt(DOOR_SENSOR_PIN));
	//Serial.print("Opened");
	byte isOpen = hwDigitalRead(DOOR_SENSOR_PIN);
	send(doorMsg.set(isOpen));

	uint32_t enteringMS = 0;
	while (enteringMS < 2000) {
		enteringMS++;
	}
	wasReport = true;

	attachInterrupt(digitalPinToInterrupt(DOOR_SENSOR_PIN), onOpen, CHANGE);
}
*/
