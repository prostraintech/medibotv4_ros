
class LED{
public:
  LED(int, int, int);//driving motor 
  void Emit(char);
  void Emit(int, int, int);
private:
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
	switch(colour){
		case 'w':
			R=255; G=255; B=255; break; //white
    case 'r':
      R=255; G=0; B=0; break; //red
    case 'g':
      R=0; G=255; B=0; break; //green
		case 'b':
			R=0; G=0; B=255; break; //blue
    case 'y':
      R=255; G=255; B=0; break; //yellow
	}
	digitalWrite(this->R, 255-R);
	digitalWrite(this->G, 255-G);
	digitalWrite(this->B, 255-B);
}

void LED::Emit(int r, int g, int b){
  digitalWrite(this->R, 255-r);
  digitalWrite(this->G, 255-g);
  digitalWrite(this->B, 255-b);
}
