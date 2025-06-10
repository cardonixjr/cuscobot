/*
  -  CONTROLE DE MOTOR DC COM MINE PONTE H L9110S -
  =================================================
  === BLOG DA ROBOTICA - www.blogdarobotica.com ===
  =================================================
  Autor: Jonas Souza
  E-mail: contato@blogdarobotica.com
  Facebook: facebook.com/blogdarobotica
  Instagram:@blogdarobotica
  YouTube: youtube.com/user/blogdarobotica
  =================================================
  === CASA DA ROBOTICA - www.casadarobotica.com ===
  =================================================
  Facebook: facebook.com/casadaroboticaoficial
  Instagram:@casadarobotica
  ==================================================
*/
int motorA_PWM = 10; //Controle de velocidade do motor A (Esquerdo)
int motorB_PWM = 11; //Controle de velocidade do motor B (Direito)
int motorA_EN = 12; //Controle de direção do motor A (Esquerdo))
int motorB_EN = 13; //Controle de direção do motor B (Direito)
int velocidade = 127; //variável para controle de velocidade de rotação dos motores,sendo 0 o valor de velocidade mínimo e 255 o valor de velocidade máxima. 
void setup(){ 
  //Configura os motores como saída
  pinMode (motorA_PWM, OUTPUT);
  pinMode (motorA_EN, OUTPUT);
  pinMode (motorB_PWM, OUTPUT);
  pinMode (motorB_EN, OUTPUT);
  delay(5000);
}
void loop(){ 
    velocidade = 127;
    analogWrite(motorA_PWM, 0); //PWM do motor esquerdo 
    analogWrite(motorB_PWM, velocidade); //PWM do motor direito 
    delay(5000); //Aguarda 5000 milissegundos

    analogWrite(motorA_PWM, velocidade); //PWM do motor esquerdo 
    analogWrite(motorB_PWM, 0); //PWM do motor direito 
    delay(5000); //Aguarda 5000 milissegundos

    velocidade = 30;
    analogWrite(motorA_PWM, velocidade); //PWM do motor esquerdo 
    analogWrite(motorB_PWM, velocidade); //PWM do motor direito 
    delay(5000); //Aguarda 5000 milissegundos

    velocidade = 200;
    analogWrite(motorA_PWM, velocidade); //PWM do motor esquerdo 
    analogWrite(motorB_PWM, velocidade); //PWM do motor direito 
    delay(5000); //Aguarda 5000 milissegundos
}
void MotorSentidoHorario(){
  digitalWrite(motorA_EN, HIGH); //Motor A. HIGH = HORARIO
  digitalWrite(motorB_EN, HIGH); //Motor B. HIGH = HORARIO
  
  analogWrite(motorA_PWM, velocidade); //PWM do motor esquerdo 
  analogWrite(motorB_PWM, velocidade); //PWM do motor direito 
}
void MotorSentidoAntiHorario(){ 
  digitalWrite(motorA_EN, LOW); //Motor A. LOW = ANTI-HORÁRIO
  digitalWrite(motorB_EN, LOW); //Motor B. LOW = ANTI-HORÁRIO
  analogWrite(motorA_PWM, velocidade); //PWM do motor esquerdo 
  analogWrite(motorB_PWM, velocidade); //PWM do motor direito 
}
