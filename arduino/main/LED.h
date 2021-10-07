
class LED{
public:
  LED(int, int, int);//driving motor 
  void Emit(char);
private:
  char colour;
  void initLEDPins();
  int R;
  int G;
  int B;
};

LED::LED(int R, int G, int B){
  this->R = R;
  this->G = G;
  this->B = B;
  LED::initLEDPins();
}

void LED::initLEDPins(){
  pinMode(this->R, OUTPUT);
  pinMode(this->G, OUTPUT);
  pinMode(this->B, OUTPUT);
  LED::Emit('w');
}
	
void LED::Emit(char colour){
	int R, G, B;
	switch(this->colour){
		case 'w':
			R=254; G=254; B=254; break; //white
		case 'b':
			R=0; G=0; B=254; break; //blue
	}
	digitalWrite(this->R, R);
	digitalWrite(this->G, G);
	digitalWrite(this->B, B);
}
