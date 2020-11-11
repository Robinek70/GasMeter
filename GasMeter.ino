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

//#define MY_SIGNING_SOFT
//#define MY_SIGNING_SOFT_RANDOMSEED_PIN A6
//#define MY_SIGNING_REQUEST_SIGNATURES

#define MY_SPECIAL_DEBUG

//#define RP_DEBUG
#define RP_SLEEP_MODE

#include <MySensors.h>
#include <myRPlibs.h>

#include <myRPcontact.h>
#include <myRPbattery.h>
#include <myRPAVRtemp.h>
#include <myRPsignal.h>
#include <myRPpulsecounter.h>

#define DOOR_SENSOR_PIN		2 
#define PULSE_SENSOR		3                  // The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)

#define PULSE_FACTOR 100                        // Number of blinks per m3 of your meter (One rotation/liter)

#define SLEEP_MODE true                        // flowvalue can only be reported when sleep mode is false.

#define MAX_FLOW 40                             // Max flow (l/min) value to report. This filters outliers.

#define CHILD_ID		102                             // Id of the sensor child
//#define RP_ID_DOOR		80                              // Id of the alarm

#define NODE_VER		F("1.3-3V" STR(MY_RF24_PA_LEVEL)"8")

uint32_t SEND_FREQUENCY =
    60000UL*5UL;           // Minimum time between send (in milliseconds). We don't want to spam the gateway.
/*
MyMessage flowMsg(CHILD_ID,V_FLOW);
MyMessage volumeMsg(CHILD_ID,V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID,V_VAR3);



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
byte pulseStatus = 0;*/

//RpBattery rpbattery(A7, INTERNAL, 110);
//RpContact contact(DOOR_SENSOR_PIN);
//RpContact contact2(4);
//Rpavrtemp avrtemp();
		

void before() {
	//attachInterrupt(digitalPinToInterrupt(PULSE_SENSOR), onPulse, CHANGE);
	//attachInterrupt(digitalPinToInterrupt(DOOR_SENSOR_PIN), onOpen, CHANGE);

//	lastSend = lastPulse = rp_now;
 //  pulseCount = oldPulseCount = 0;
   new RpAVRtemp();
   new RpSignal();
   new RpContact(DOOR_SENSOR_PIN);

   /*(new RpBattery(A7, DEFAULT, 500))
		->setBattery(240,500)
		->setDivider(0,1);*/
   (new RpBattery(A7, INTERNAL, 110))
		->setBattery(220,290);

   new myRpPulseCounter(PULSE_SENSOR);


   rp_before();
}

void setup()
{
	rp_setup();
}

void presentation()
{
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo(F("Gas Meter"), NODE_VER);

	rp_presentation();
}

void loop()
{
	rp_loop();

	/*if(!rp_requreReinit) {

		(void)_sendRoute(build(_msg, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
			I_POST_SLEEP_NOTIFICATION).set(SEND_FREQUENCY));

		(void)_sendRoute(build(_msg, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
			I_PRE_SLEEP_NOTIFICATION).set((uint32_t)MY_SMART_SLEEP_WAIT_DURATION_MS));
			
		wait(MY_SMART_SLEEP_WAIT_DURATION_MS);
	}*/
	//Serial.println(rp_now);
    if (SLEEP_MODE) {
		SEND_FREQUENCY = rp_sleepTime ;
		int8_t ii = sleep(digitalPinToInterrupt(DOOR_SENSOR_PIN), CHANGE, digitalPinToInterrupt(PULSE_SENSOR), CHANGE, SEND_FREQUENCY);
		if(ii==-1) {
			rp_add_sleep_time += SEND_FREQUENCY;
		} else {
			rp_add_sleep_time += 10000;
		}

		rp_requreReinit = true;

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

void receive(const MyMessage &message)
{   
	if (message.isAck()) {
		return;
	}

	rp_receive(message);
}

/*void onPulse()
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
}*/
/*
void checkPulse() {
	byte isClose = hwDigitalRead(PULSE_SENSOR);
	if(pulseStatus != isClose) {
		pulseStatus = isClose;
		pulseCount++;
		//wasReport = true;
	}
}*/


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

//ISR(WDT_vect){	
//}
