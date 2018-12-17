// Motor control
void motor_control(int pwm_r, int pwm_l)
{
  // Forward
  if(pwm_r >= 0 and pwm_l >= 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, pwm_l);
    analogWrite(ENB, pwm_r);
  }
  // Left
  else if(pwm_r >= 0 and pwm_l <= 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, abs(pwm_l));
    analogWrite(ENB, pwm_r);
  }
  // Right
  else if(pwm_r <= 0 and pwm_l >= 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, pwm_l);
    analogWrite(ENB, abs(pwm_r));
  }
  // Backward
  else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, abs(pwm_l));
    analogWrite(ENB, abs(pwm_r));
  }
}
