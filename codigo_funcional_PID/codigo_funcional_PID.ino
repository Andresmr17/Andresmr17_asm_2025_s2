/***************************************************
 * RÁPIDO 255PWM + PID FINAL PRECISO
 * >7cm: 255PWM (MUY RÁPIDO) | <7cm: PID preciso
 * Modelo orden 2: G(s)=15/[s(0.3s+1)(0.05s+1)]
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

// UMBRALES CLAROS
const float stopDist = 3.0f;    // STOP
const float pidDist = 7.0f;     // INICIO PID
const float fastPWM = 255;      // MAXIMO RÁPIDO

// PID SOLO en zona final (<7cm)
const float Ts = 0.050f;        // 20Hz
const float Kp = 4.0f;
const float Ki = 12.0f;
const float Kd = 0.20f;

float error_pid = 0, integral_pid = 0, prev_error_pid = 0, u_pid = 0;
unsigned long lastPID = 0;
unsigned long lastBtn = 0;
int mode = 0;

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

  Serial.println(F("=== HIBRIDO: RÁPIDO 255PWM + PID PRECISO ==="));
  Serial.println(F(">7cm: 255PWM | <7cm: PID Kp=4 Ki=12 Kd=0.2"));
  Serial.println(F("Modelo orden 2: Polos 0,-3.33,-20 | Ts=50ms"));
}

void loop() {
  unsigned long now = millis();
  
  // BOTONES
  if (digitalRead(BTN_ABRIR) == LOW && now - lastBtn > 500) {
    lastBtn = now; mode = 1; integral_pid = 0; prev_error_pid = 0;
    Serial.println(F("ABRIR: DERECHO→3cm"));
  }
  if (digitalRead(BTN_CERRAR) == LOW && now - lastBtn > 500) {
    lastBtn = now; mode = -1; integral_pid = 0; prev_error_pid = 0;
    Serial.println(F("CERRAR: IZQUIERDO→3cm"));
  }

  // CONTROL 100ms (como tu código original)
  static unsigned long lastControl = 0;
  if (now - lastControl >= 100) {
    lastControl = now;
    
    float dist_I = medirDistancia(TRIG_I, ECHO_I);
    float dist_D = medirDistancia(TRIG_D, ECHO_D);
    float pos = (mode == 1) ? dist_D : dist_I;
    
    Serial.print(F("Mode=")); Serial.print(mode);
    Serial.print(F(" I=")); Serial.print(dist_I,1);
    Serial.print(F(" D=")); Serial.print(dist_D,1);
    Serial.print(F(" Pos=")); Serial.print(pos,1); Serial.print(F("cm"));

    if (mode == 0) {
      motorStop(); Serial.println(); return;
    }

    // ========== FASE 1: RÁPIDO 255PWM (>7cm) ==========
    if (pos > pidDist) {
      Serial.print(F(" → RAPIDO ")); Serial.println(fastPWM);
      if (mode == 1) motorDerecha(fastPWM);
      else motorIzquierda(fastPWM);
    } 
    // ========== FASE 2: PID PRECISO (<7cm) ==========
    else if (pos > stopDist) {
      if (now - lastPID >= 50) {
        lastPID = now;
        
        error_pid = stopDist - pos;  // Error POSITIVO = acercarse más
        integral_pid += error_pid * Ts;
        integral_pid = constrain(integral_pid, -50, 50);
        
        float derivada = (error_pid - prev_error_pid) / Ts;
        u_pid = Kp * error_pid + Ki * integral_pid + Kd * derivada;
        u_pid = constrain(u_pid, 80, 255);
        
        prev_error_pid = error_pid;
        
        Serial.print(F(" [PID e=")); Serial.print(error_pid,1);
        Serial.print(F(" i=")); Serial.print(integral_pid,1);
        Serial.print(F(" u=")); Serial.print((int)u_pid,0);
        
        if (mode == 1) motorDerecha((int)u_pid);
        else motorIzquierda((int)u_pid);
      }
      Serial.println(F("]"));
    } 
    // ========== STOP ==========
    else {
      Serial.println(F(" → STOP OK!"));
      motorStop();
      mode = 0;
    }
    Serial.println();
  }
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
