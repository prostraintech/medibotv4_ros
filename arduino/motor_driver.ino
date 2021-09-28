void Move(double LH_PWM, double RH_PWM){
    if(LH_PWM>0){//left wheel forward
      analogWrite(LH_D1,LH_PWM);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,LOW);
    }
    else if(LH_PWM<0){//left wheel backward
      analogWrite(LH_D1,abs(LH_PWM));
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,HIGH);
    }
    else{//left wheel stop
      analogWrite(LH_D1,0);
      digitalWrite(LH_D2,LOW);
      digitalWrite(BR,HIGH);
    }

    if(RH_PWM>0){//right wheel forward
      analogWrite(RH_D1,RH_PWM);
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
    }
    else if(RH_PWM<0){//right wheel backward
      analogWrite(RH_D1,abs(RH_PWM));
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,HIGH);
    }
    else{//right wheel stop
      analogWrite(RH_D1,0);
      digitalWrite(RH_D2,LOW);
      digitalWrite(BR,HIGH);
    }
}