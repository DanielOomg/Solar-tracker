// Seguidor solar 2 ejes - UNO R4 Minima. Motor H: pines 8-11, motor V: pines 2-5 (28BYJ-48 + ULN2003).
// Serie 115200: c=calibrar (linterna de frente) h/v=invertir eje r=centro 9=subir al tope 0=centro u/d=subir/bajar (prueba)
const int LDR[4] = {A3, A1, A2, A0};
const char* NOM[4] = {"SI", "SD", "II", "ID"};
const int   PASOS_VUELTA = 2048, RPM_H = 15, RPM_V = 10, RAMPA_H = 10, RAMPA_V = 25;
const float SAT = 1018, LUZ_MIN = 80, ALFA = 0.35, TOL_STOP = 1.5, TOL_GO = 3.0, GAN_MAX = 3.5;
const float GRADOS_ARRIBA = 180, GRADOS_ABAJO = 79.1, RELACION_V = 1.0;
const bool  SOLTAR_V = false;
const long  LIM_ARRIBA = (long)(GRADOS_ARRIBA * PASOS_VUELTA / 360.0 * RELACION_V + 0.5);
const long  LIM_ABAJO  = (long)(GRADOS_ABAJO * PASOS_VUELTA / 360.0 * RELACION_V + 0.5);

bool invH = true, invV = false, ini = false, movH = false, movV = false, sinCal[4];
float gan[4] = {1, 1, 1, 1}, vs[4];
unsigned long tPrint = 0, noV = 0;

struct Motor { byte p[4]; long pend, pos; byte fase; unsigned long t; bool on; unsigned long dt; bool soltar; int k, rampa; };
Motor mH = {{8, 9, 10, 11}, 0, 0, 0, 0, false, 60000000UL / ((unsigned long)PASOS_VUELTA * RPM_H), true, 0, RAMPA_H};
Motor mV = {{2, 3, 4, 5}, 0, 0, 0, 0, false, 60000000UL / ((unsigned long)PASOS_VUELTA * RPM_V), SOLTAR_V, 0, RAMPA_V};
const byte SEQ[4][4] = {{1, 1, 0, 0}, {0, 1, 1, 0}, {0, 0, 1, 1}, {1, 0, 0, 1}};

// Da un paso del motor si toca (sin bloquear), con arranque suave
void servir(Motor &m) {
  unsigned long ahora = micros();
  if (!m.pend) {
    if (m.soltar && m.on && ahora - m.t > 3 * m.dt) { for (byte i = 0; i < 4; i++) digitalWrite(m.p[i], LOW); m.on = false; }
    return;
  }
  if (ahora - m.t > 200000UL) m.k = 0;
  unsigned long dt = m.k < m.rampa ? m.dt * 3 - m.dt * 2 * m.k / m.rampa : m.dt;
  if (ahora - m.t < dt) return;
  m.k++; m.t = ahora;
  int d = m.pend > 0 ? 1 : -1;
  m.fase = (m.fase + d) & 3; m.pend -= d; m.pos += d;
  for (byte i = 0; i < 4; i++) digitalWrite(m.p[i], SEQ[m.fase][i]);
  m.on = true;
}

// Espera moviendo los motores
void esperar(unsigned long us) { unsigned long t0 = micros(); do { servir(mH); servir(mV); } while (micros() - t0 < us); }

// Promedio de n lecturas
float leer(int pin, int n, unsigned long us) {
  long s = 0;
  for (int k = 0; k < n; k++) { s += analogRead(pin); esperar(us); }
  return s / (float)n;
}

// Pasos proporcionales al error (4 a tope)
int pasos(float e, int tope) { return constrain((int)(fabs(e) + 0.1 * e * e), 4, tope); }

// Iguala los 4 LDR con la linterna de frente; rechaza uno que lea muy bajo
void calibrar() {
  float v[4], s = 0; int n = 0, rech = 0;
  for (int i = 0; i < 4; i++) { v[i] = leer(LDR[i], 32, 5000); if (v[i] > 5 && v[i] < SAT) { s += v[i]; n++; } }
  float p0 = n ? s / n : 0;
  if (p0 < 2 * LUZ_MIN) { Serial.println("Poca luz: linterna de frente y repite c"); return; }
  for (int i = 0; i < 4; i++) if (v[i] < p0 / GAN_MAX) rech++;
  if (rech > 1) { Serial.println("Luz despareja: linterna de FRENTE y centrada, repite c"); return; }
  s = 0; n = 0;
  for (int i = 0; i < 4; i++) { sinCal[i] = v[i] < p0 / GAN_MAX; if (!sinCal[i] && v[i] < SAT) { s += v[i]; n++; } }
  float p = n ? s / n : 0;
  Serial.print("Calibrado:");
  for (int i = 0; i < 4; i++) {
    gan[i] = (!sinCal[i] && v[i] > 5 && v[i] < SAT && p > 0) ? constrain(p / v[i], 0, GAN_MAX) : 1;
    Serial.print(' '); Serial.print(NOM[i]); Serial.print('='); Serial.print(gan[i], 2);
    if (sinCal[i]) Serial.print("(rechazado)");
  }
  Serial.println(); ini = false;
}

// Comandos por el Monitor Serie
void serie() {
  while (Serial.available()) {
    char c = tolower(Serial.read());
    long s = invV ? -1 : 1;
    if (c == 'c') calibrar();
    else if (c == 'h') { invH = !invH; Serial.println(invH ? "invH=1" : "invH=0"); }
    else if (c == 'v') { invV = !invV; Serial.println(invV ? "invV=1" : "invV=0"); }
    else if (c == 'r') { mH.pos = mV.pos = 0; Serial.println("centro"); }
    else if (c == '9') { mV.pend = s * LIM_ARRIBA - mV.pos; noV = millis() + 6000; Serial.println("subiendo al tope..."); }
    else if (c == '0') { mV.pend = -mV.pos; noV = millis() + 6000; Serial.println("volviendo al centro..."); }
    else if (c == 'u' || c == 'd') { mV.pend += (c == 'u' ? 1 : -1) * s * 128; noV = millis() + 3000; }
  }
}

void setup() {
  Serial.begin(115200); analogReadResolution(10);
  for (byte i = 0; i < 4; i++) { pinMode(mH.p[i], OUTPUT); pinMode(mV.p[i], OUTPUT); digitalWrite(mH.p[i], LOW); digitalWrite(mV.p[i], LOW); }
  delay(500);
  Serial.println("c=calibrar h/v=invertir r=centro 9=subir al tope 0=centro u/d=probar motor vertical");
}

// Lee los LDR, estima uno malo con los otros 3 y mueve cada eje hacia donde hay mas luz
void loop() {
  serie();
  float r[4], prom = 0;
  for (int i = 0; i < 4; i++) { r[i] = leer(LDR[i], 10, 100); prom += r[i] / 4; }

  bool malo[4]; int nm = 0, im = 0;
  for (int i = 0; i < 4; i++) {
    malo[i] = sinCal[i] || (r[i] < 5 && prom > 2 * LUZ_MIN);
    if (malo[i]) { nm++; im = i; }
    else { float g = r[i] * gan[i]; vs[i] = ini ? vs[i] + ALFA * (g - vs[i]) : g; }
  }
  ini = true;
  if (nm == 1) { float e = vs[im ^ 1] + vs[im ^ 2] - vs[3 - im]; vs[im] = e > 0 ? e : 0; }
  else if (nm > 1) for (int i = 0; i < 4; i++) if (malo[i]) vs[i] = 0;

  float luz = 0, mx = 0, rH = 0, rV = 0;
  for (int i = 0; i < 4; i++) { luz += vs[i]; if (vs[i] > mx) mx = vs[i]; }
  luz /= 4;
  bool act = mx > LUZ_MIN && luz > 15 && nm <= 1;
  if (act) {
    for (int i = 0; i < 4; i++) {
      float e = (vs[i] - luz) * 50 / luz;
      if (e > 0) { rH += (i % 2) ? -e : e; rV += (i < 2) ? e : -e; }
    }
    movV = fabs(rV) > (movV ? TOL_STOP : TOL_GO);
    movH = fabs(rH) > (movH ? TOL_STOP : TOL_GO);
    if (movV && !mV.pend && millis() > noV) {
      long s = invV ? -1 : 1;
      long arriba = constrain(s * mV.pos + (rV > 0 ? 1 : -1) * pasos(rV, 80), -LIM_ABAJO, LIM_ARRIBA);
      mV.pend = s * arriba - mV.pos;
    }
    if (movH && !mH.pend) mH.pend = ((rH > 0) != invH ? 1 : -1) * pasos(rH, 120);
  } else movH = movV = false;

  if (millis() - tPrint > 500) {
    tPrint = millis();
    for (int i = 0; i < 4; i++) { Serial.print(NOM[i]); Serial.print(malo[i] ? "! " : ":"); Serial.print((int)vs[i]); Serial.print(' '); }
    Serial.print("| V "); Serial.print(rV, 1); Serial.print(" H "); Serial.print(rH, 1);
    Serial.print(" | arriba "); Serial.print((invV ? -mV.pos : mV.pos) * 360.0 / PASOS_VUELTA / RELACION_V, 0);
    Serial.println(!act ? (nm > 1 ? "deg PARADO: 2+ LDR malos (manda c con luz de frente)" : "deg PARADO: poca luz") : (movV || movH) ? "deg buscando" : "deg alineado");
  }
  esperar(3000);
}
