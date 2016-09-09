#include <Wire.h>

//#define DEBUG

#define I2C_ADDR 0x08

#define ROTARY_INT 1
#define LIDAR_PIN 6
#define LIDAR_EN_PIN 5

#define MODE_TICKS 0
#define MODE_RANGE 1

volatile uint8_t TICKS = 0; // where rotation ticks
volatile uint8_t RANGE = 0; // range in decimeters

void resetLidar()
{
#ifdef DEBUG
  Serial.print("Resetting LIDAR...");
#endif
	digitalWrite(LIDAR_EN_PIN, LOW);
	delay(1000);
	digitalWrite(LIDAR_EN_PIN, HIGH);
	delay(1000);
#ifdef DEBUG
	Serial.println("done");
#endif
}
//-----------------------------------------------------------------------------
void setup() {
#ifdef DEBUG
	Serial.begin(9600);
	Serial.println("Hello from Kurzbot encoder");
#endif

	// put your setup code here, to run once:
	DDRB &= B11001111;

	attachInterrupt(ROTARY_INT, wheelInt, RISING);

	pinMode(LIDAR_EN_PIN, OUTPUT);
	pinMode(LIDAR_PIN, INPUT);

	resetLidar();

	Wire.begin(I2C_ADDR);
	Wire.onRequest(i2cSendData);
	Wire.onReceive(i2cReceive);
}
//-----------------------------------------------------------------------------
void i2cReceive(int bytes)
{
#ifdef DEBUG
  Serial.print("i2c data <- "); Serial.println(bytes, DEC);
#endif

	if(bytes == 2)
	{
#ifdef DEBUG
	Serial.println("Resetting");
#endif

		uint8_t mode = Wire.read();
		TICKS = 0;

		resetLidar();
	}
}
//-----------------------------------------------------------------------------
void i2cSendData()
{
	uint8_t old_ticks = TICKS;

	// merge the ticks and range into a 2 byte buffer
	// MSB -> range, LSB -> ticks
	uint8_t data[2] = { old_ticks, RANGE };
	Wire.write(data, 2);

#ifdef DEBUG
	if(data[0] != 0)
	{
		Serial.print(data[0], DEC);
		Serial.print(" ");
		Serial.println(data[1], DEC);
	}
#endif

	// don't reset the tick counter if we haven't accumulated
	// any rotations yet
	if(old_ticks > 0)
	{
		TICKS = 0;
	}
}
//-----------------------------------------------------------------------------
void wheelInt()
{
	uint8_t b_set = PINB & B00010000;   // read the input pin

	// and adjust counter + if A leads B
	TICKS++;
#ifdef DEBUG
	Serial.print("ticks: "); Serial.println(TICKS, DEC);
#endif
}
//-----------------------------------------------------------------------------
void loop()
{
#ifdef DEBUG
	uint8_t last_range = RANGE;
#endif

	unsigned long pulse = pulseIn(LIDAR_PIN, HIGH);

	if(pulse == 0)
	{
		resetLidar();
		return;
	}

	RANGE = pulse / 100;
#ifdef DEBUG
	if(last_range != RANGE)
	{
		Serial.print("dist (dM): "); Serial.println(RANGE, DEC);
		last_range = RANGE;
	}
#endif
}
