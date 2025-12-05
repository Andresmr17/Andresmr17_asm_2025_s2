/***************************************************
 * SIN PID: 4 SEGUNDOS MÁXIMA VELOCIDAD por botón
 * ABRIR: motorDerecha 4s | CERRAR: motorIzquierda 4s
 ***************************************************/

#define AIN1 5
#define AIN2 6
#define PWMA 9
#define STBY 10
#define BTN_ABRIR 2
#define BTN_CERRAR 3
#define TRIG_I 11
#define ECHO_I 12
#define TRIG_D 7
#define ECHO_D 8

const unsigned long TIEMPO_MOVIMIENTO = 3000;  // 4 segundos
const int VELOCIDAD_MAX = 255;

unsigned long tiempoInicio = 0;
bool motorActivo = false;
int direccion = 0;  // 1=derecha, -1=izquierda, 0=stop

float medirDistancia(int trigPin, int echoPin);
void motorDerecha(int pwm);
void motorIzquierda(int pwm);
void motorStop();

void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT); pinMode(STBY, OUTPUT);
  pinMode(BTN_ABRIR, INPUT_PULLUP); pinMode(BTN_CERRAR, INPUT_PULLUP);
  pinMode(TRIG_I, OUTPUT); pinMode(ECHO_I, INPUT); pinMode(TRIG_D, OUTPUT); pinMode(ECHO_D, INPUT);
  motorStop();

  Serial.println(F("=== SIN PID: 4 SEGUNDOS MÁXIMA VELOCIDAD ==="));
  Serial.println(F("ABRIR: 4s DERECHA | CERRAR: 4s IZQUIERDA"));
}

void loop() {
  unsigned long now = millis();
  
  // BOTONES: Iniciar movimiento 4s
  static unsigned long lastBtn = 0;
  if (digitalRead(BTN_ABRIR) == LOW && now - lastBtn > 500) {
    lastBtn = now;
    direccion = 1;
    tiempoInicio = now;
    motorActivo = true;
    Serial.println(F("ABRIR: 4s DERECHA 255PWM"));
  }
  if (digitalRead(BTN_CERRAR) == LOW && now - lastBtn > 500) {
    lastBtn = now;
    direccion = -1;
    tiempoInicio = now;
    motorActivo = true;
    Serial.println(F("CERRAR: 4s IZQUIERDA 255PWM"));
  }

  // CONTROL SIMPLE: 4 SEGUNDOS o STOP manual
  if (motorActivo) {
    if (now - tiempoInicio >= TIEMPO_MOVIMIENTO) {
      // FIN AUTOMÁTICO 4s
      motorStop();
      motorActivo = false;
      direccion = 0;
      Serial.println(F("4 SEGUNDOS COMPLETADOS - STOP"));
    } else {
      // MOVER MÁXIMA VELOCIDAD
      float dist_I = medirDistancia(TRIG_I, ECHO_I);
      float dist_D = medirDistancia(TRIG_D, ECHO_I);
      
      Serial.print(F("t=")); Serial.print((now - tiempoInicio)/1000.0,1);
      Serial.print(F("s | I=")); Serial.print(dist_I,1);
      Serial.print(F(" D=")); Serial.print(dist_D,1);
      
      if (direccion == 1) {
        motorDerecha(VELOCIDAD_MAX);
        Serial.println(F(" | DERECHA 255PWM"));
      } else {
        motorIzquierda(VELOCIDAD_MAX);
        Serial.println(F(" | IZQUIERDA 255PWM"));
      }
    }
  } else {
    motorStop();
  }
  
  delay(100);  // Control 100ms
}

float medirDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(3);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long dur = pulseIn(echoPin, HIGH, 40000UL);
  if (dur == 0) return 999.0f;
  float dist = (dur * 0.0343f) / 2.0f;
  return constrain(dist, 2.0f, 400.0f);
}

void motorDerecha(int pwm) {
  digitalWrite(STBY, HIGH); digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  analogWrite(PWMA, constrain(pwm, 0, 255));
}

void motorIzquierda(int pwm) {
  digitalWrite(STBY, HIGH); digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, constrain(pwm, 0, 255));
}

void motorStop() {
  analogWrite(PWMA, 0); digitalWrite(STBY, LOW);
}
