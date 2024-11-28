#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

#define ina1 7   // pin 1 de dirección del Motor Izquierdo
#define ina2 4   // pin 2 de dirección del Motor Izquierdo
#define ena 6    // pin PWM del Motor Izquierdo

#define inb1 8   // pin 1 de dirección del Motor Derecho
#define inb2 9   // pin 2 de dirección del Motor Derecho
#define enb 5    // pin PWM del Motor Derecho

#define HDE A7   // Sensor Derecho (analógico)

int base = 50;
float Kprop = 1.5;
float Kderiv = 9;
float Kinte = 0.0;
int setpoint = 2500;  // Posición central
int last_error = 0;
int pot_limite = 100; // Límite máximo de potencia
int margen_inferior = 2300; // Límite inferior del margen de error
int margen_superior = 2700; // Límite superior del margen de error

int countDerecho = 0;      // Contador de marcas para el lado derecho
int umbral = 700;          // Umbral para detectar una marca blanca (ajustar según sea necesario)
bool hde_last_state = false; // Estado previo del sensor derecho

void setup() {
    pinMode(ena, OUTPUT);
    pinMode(ina1, OUTPUT);
    pinMode(ina2, OUTPUT);

    pinMode(enb, OUTPUT);
    pinMode(inb1, OUTPUT);
    pinMode(inb2, OUTPUT);

    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
    qtr.setEmitterPin(11);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    for (uint16_t i = 0; i < 200; i++) {
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW);
    Serial.begin(9600);
}

void loop() {
    // Si el contador del sensor derecho alcanza 4, detener motores
    if (countDerecho >= 4) {
        Motores(0, 0);  // Detener los motores
        Serial.println("Detenido tras detectar 4 marcas en el sensor derecho.");
        return;
    }

    uint16_t position = qtr.readLineWhite(sensorValues);
    int line_position = position;
    int Correction_power = 0;  // Inicializamos a cero

    // Contar marcas blancas solo en el sensor derecho
    contarMarcaDerecha();

    // Calcular la corrección si la posición está fuera del margen
    if (line_position < margen_inferior || line_position > margen_superior) {
        Correction_power = PIDLambo(line_position, Kprop, Kderiv, Kinte);
    }

    int leftPower = base;
    int rightPower = base;

    // Aplicar la corrección si estamos fuera del margen
    if (line_position > setpoint && line_position > margen_superior) {
        leftPower -= Correction_power;  // Reducir la potencia del motor izquierdo
    } else if (line_position < setpoint && line_position < margen_inferior) {
        rightPower -= Correction_power;  // Reducir la potencia del motor derecho
    }

    leftPower = constrain(leftPower, 0, 255);
    rightPower = constrain(rightPower, 0, 255);

    Motores(leftPower, rightPower);

    Serial.print("Posición: ");
    Serial.print(position);
    Serial.print(" Corrección: ");
    Serial.print(Correction_power);
    Serial.print(" | Contador Derecho: ");
    Serial.print(countDerecho);
    Serial.print(" | Lectura HDE: ");
    Serial.println(analogRead(HDE));  // Mostrar el valor analógico actual del sensor derecho
}

void contarMarcaDerecha() {
    // Leer el valor analógico del sensor derecho
    int hde_value = analogRead(HDE);

    // Detectar una transición de fondo oscuro a marca blanca en el sensor derecho
    bool hde_current_state = hde_value > umbral;
    if (hde_current_state && !hde_last_state) {
        countDerecho++;  // Incrementar el contador derecho
        Serial.println("Marca derecha detectada.");
    }

    // Actualizar el último estado del sensor derecho
    hde_last_state = hde_current_state;
}

int PIDLambo(int POS, float Kp, float Kd, float Ki) {
    int error = POS - setpoint;
    int derivative = error - last_error;
    last_error = error;

    // Ajuste de potencia proporcional al error, para un efecto de "fade"
    int pot_giro = abs(error * Kp + derivative * Kd);

    // Limitar la potencia de giro al rango máximo establecido
    pot_giro = map(pot_giro, 0, 5000, 0, pot_limite);  // Mapear entre 0 y pot_limite

    return pot_giro;
}

void MotorIz(int value) {
    digitalWrite(ina1, LOW);
    digitalWrite(ina2, HIGH);
    analogWrite(ena, value);
}

void MotorDe(int value) {
    digitalWrite(inb1, HIGH);
    digitalWrite(inb2, LOW);
    analogWrite(enb, value);
}

void Motores(int left, int right) {
    MotorIz(left);
    MotorDe(right);
}
