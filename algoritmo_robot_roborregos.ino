const int PWM2A = 11;
const int PWM2B = 3;
const int PWM0A = 6;
const int PWM0B = 5;
const int DIR_CLK = 4;
const int DIR_EN = 7;
const int DATA = 8;
const int DIR_LATCH = 12;


const int Move_Forward = 39;
const int Move_Backward = 216;
const int Left_Move = 116;
const int Right_Move = 139;
const int Stop = 0;

// pins para los sensores
const int TRIG_PIN = 2;
const int ECHO_PINS[4] = {A0, A1, A2, A3};

// configuracion del laberinto
const int FILAS = 6;
const int COLS = 6;
const int INICIO_F = FILAS - 1;   
const int INICIO_C = 0;
const int META_F = 0;             
const int META_C = COLS - 1;

// se tiene que calibrar para ajustarlo a la pista
const int VELOCIDAD = 160;               
const unsigned long MS_CELDA_LONG = 700; 
const unsigned long MS_CELDA_LAT = 900;  
const unsigned long PAUSA_MS = 250;      
const int DIST_PARED_CM = 18;            
const int DIST_FRENO_CM = 7;             

// direcciones
const int DF[4] = {-1, 0, 1, 0};
const int DC[4] = {0, 1, 0, -1};
const int CODIGO_MOV[4] = {Move_Forward, Right_Move, Move_Backward, Left_Move};
const char NOMBRE_DIR[4] = {'N', 'E', 'S', 'O'};


const int PRIORIDAD[4] = {0, 1, 3, 2};


bool visitado[FILAS][COLS];
byte pilaDir[FILAS * COLS];   
int tope = 0;                 
int fila = INICIO_F;
int col = INICIO_C;
bool terminado = false;
unsigned int movimientos = 0;

void Motor(int Dir, int s1, int s2, int s3, int s4) {
  analogWrite(PWM2A, s1);
  analogWrite(PWM2B, s2);
  analogWrite(PWM0A, s3);
  analogWrite(PWM0B, s4);

  digitalWrite(DIR_LATCH, LOW);
  shiftOut(DATA, DIR_CLK, MSBFIRST, Dir);
  digitalWrite(DIR_LATCH, HIGH);
}

void detener() {
  Motor(Stop, 0, 0, 0, 0);
}

// para los sensores
int medirUnaVez(int sensor) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long dur = pulseIn(ECHO_PINS[sensor], HIGH, 25000UL);
  if (dur == 0) return 999;
  return (int)(dur / 58UL);
}


int distanciaCm(int sensor) {
  int a = medirUnaVez(sensor); delay(15);
  int b = medirUnaVez(sensor); delay(15);
  int c = medirUnaVez(sensor); delay(15);
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

void leerParedes(bool pared[4]) {
  for (int d = 0; d < 4; d++) {
    pared[d] = distanciaCm(d) < DIST_PARED_CM;
  }
}


void moverCelda(int d) {
  unsigned long duracion = (d == 0 || d == 2) ? MS_CELDA_LONG : MS_CELDA_LAT;

  Motor(CODIGO_MOV[d], VELOCIDAD, VELOCIDAD, VELOCIDAD, VELOCIDAD);
  unsigned long t0 = millis();
  while (millis() - t0 < duracion) {
    if (medirUnaVez(d) < DIST_FRENO_CM) break;   
    delay(20);
  }
  detener();
  delay(PAUSA_MS);

  fila += DF[d];
  col += DC[d];
  movimientos++;

  Serial.print("Mov ");
  Serial.print(NOMBRE_DIR[d]);
  Serial.print(" -> (");
  Serial.print(fila);
  Serial.print(", ");
  Serial.print(col);
  Serial.println(")");
}

bool dentroDelMapa(int f, int c) {
  return f >= 0 && f < FILAS && c >= 0 && c < COLS;
}


void imprimirCamino() {
  Serial.println();
  Serial.print("Meta alcanzada en ");
  Serial.print(movimientos);
  Serial.println(" movimientos (incluye retrocesos).");
  Serial.print("Camino desde el inicio (");
  Serial.print(tope);
  Serial.print(" celdas): ");
  for (int i = 0; i < tope; i++) {
    Serial.print(NOMBRE_DIR[pilaDir[i]]);
    Serial.print(' ');
  }
  Serial.println();
}

void celebrar() {
  const int Right_Rotate = 149;
  Motor(Right_Rotate, VELOCIDAD, VELOCIDAD, VELOCIDAD, VELOCIDAD);
  delay(1500);
  detener();
}


void setup() {
  pinMode(DIR_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(DIR_EN, OUTPUT);
  pinMode(DIR_LATCH, OUTPUT);
  pinMode(PWM0B, OUTPUT);
  pinMode(PWM0A, OUTPUT);
  pinMode(PWM2A, OUTPUT);
  pinMode(PWM2B, OUTPUT);
  digitalWrite(DIR_EN, LOW);  

  pinMode(TRIG_PIN, OUTPUT);
  for (int i = 0; i < 4; i++) pinMode(ECHO_PINS[i], INPUT);

  Serial.begin(9600);
  detener();

  for (int f = 0; f < FILAS; f++)
    for (int c = 0; c < COLS; c++)
      visitado[f][c] = false;
  visitado[fila][col] = true;

  Serial.println("Robot DFS listo. Arranca en 3 s...");
  delay(3000);
}

void loop() {
  if (terminado) return;


  if (fila == META_F && col == META_C) {
    detener();
    imprimirCamino();
    celebrar();
    terminado = true;
    return;
  }

  
  bool pared[4];
  leerParedes(pared);

  
  for (int i = 0; i < 4; i++) {
    int d = PRIORIDAD[i];
    int nf = fila + DF[d];
    int nc = col + DC[d];
    if (!pared[d] && dentroDelMapa(nf, nc) && !visitado[nf][nc]) {
      moverCelda(d);
      visitado[fila][col] = true;
      pilaDir[tope++] = d;
      return;
    }
  }


  if (tope == 0) {
    detener();
    Serial.println("No hay camino hasta la meta.");
    terminado = true;
    return;
  }
  int d = pilaDir[--tope];
  Serial.println("Callejon sin salida, retrocediendo...");
  moverCelda((d + 2) % 4);
}
