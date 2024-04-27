const int muxSIG = A0;
const int muxS0 = 49;
const int muxS1 = 50;
const int muxS2 = 51;
const int muxS3 = 52;
const int muxEN = 53;

int SetMuxChannel (byte channel)
{
  digitalWrite (muxS0, bitRead (channel, 0)); 
  digitalWrite (muxS1, bitRead (channel, 1)); 
  digitalWrite (muxS2, bitRead(channel, 2)); 
  digitalWrite(muxS3, bitRead(channel, 3));
}

void setup()
{
  pinMode (muxEN, OUTPUT);
  pinMode (muxSIG, OUTPUT);
  pinMode (muxS0, OUTPUT);
  pinMode (muxS1, OUTPUT);
  pinMode (muxS2, OUTPUT);
  pinMode (muxS3, OUTPUT);
  digitalWrite (muxEN, LOW);
  Serial.begin(115200);
}

void loop()
{
  SetMuxChannel(5);
  int val = analogRead(muxSIG);
  Serial.println(val);
  delay(100);

}