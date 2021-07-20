#include <thread>
#include <mutex>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <wiringPi.h>
#include <iostream>
#include <filesystem>
#include <sys/stat.h>

const int P_RED = 12;
const int P_YELLOW = 13;
const int P_GREEN = 14;
const int P_STEP_MOTOR[4] = {6, 10, 11, 31};
const unsigned int delayTime = 1000 * 1000; // usleep ist in Nanosekunden, weshalb 1000x1000 1s ergibt.

// 2 States für grün, da auf grün einmal grün und gelb und einmal grün, gelb und rot folgen.
enum LED_MODE {
    NONE,
    GREEN90,
    GREEN_YELLOW,
    GREEN270,
    ALL
};

std::mutex degreeMutex;
std::mutex ledMutex;

#pragma region StepMotor

/**
 * Lässt den Schrittmotor um 2 Runden drehen.
 *
 * @param degree die aktuelle Gradzahl des Schrittmotors.
 */
void stepMotor(double *degree) {
    // Wir machen 8 Durchläufe, da wir uns in jeweils 512 mal in 90/512° Schritten bewegen. (8 * 90  = 720°)
    for (int i = 0; i < 8; i++) {
        {
            // Das degreeMutex muss gelockt werden, da wir degree umändern.
            std::lock_guard<std::mutex> degreeLock(degreeMutex);
            *degree += 90;

            for (int j = 0; j < 512; j++) { // 512 * 0,17578125° = 90°
                int index = j % 4;  // Es gibt 4 PINS, weshalb die Zahl nicht größer werden darf.

                // Setzt den letzten Motorschritt auf 0.
                if (index == 0) {
                    digitalWrite(P_STEP_MOTOR[3], LOW);
                } else {
                    digitalWrite(P_STEP_MOTOR[index - 1], LOW);
                }
                digitalWrite(P_STEP_MOTOR[index], HIGH); // Setzt den aktuellen Motorschritt auf 1.
                usleep(3000); // Verzögerung für Trägheitsmoment des Rotors
            }
        }
        usleep(delayTime); // Verzögerung für Kamera und die LEDs
    }
}

#pragma endregion StepMotor

#pragma region LED

/**
 * Schaltet alle LEDs aus
 */
void resetLEDS() {
    digitalWrite(P_GREEN, LOW);
    digitalWrite(P_YELLOW, LOW);
    digitalWrite(P_RED, LOW);
}

/**
 * Schaltet die LEDs in Abhängigkeit von der Gradzahl an und speichert die eingeschalteten LEDs in mode.
 *
 * @param degree die Gradzahl des Schrittmotors
 * @param mode der Modus der LED Lampen, welcher angibt, welche LEDs leuchten sollen
 */
void led(double *degree, LED_MODE *mode) {
    LED_MODE lastMode = ALL;

    while (true) {
        {
            // ledMutex muss gelockt werden, da mode später geschrieben wird.
            std::lock_guard<std::mutex> ledLock(ledMutex);
            // degreeMutex muss gelockt werden, da degree später gelesen wird.
            std::lock_guard<std::mutex> degreeLock(degreeMutex);

            // Schaltet alle LEDs aus, falls welche eingeschaltet sind.
            if (*mode != NONE) {
                *mode = NONE;
                resetLEDS();
            }

            if (*degree >= 90) { // überprüft die Grad
                *degree -= 90;  // Reduziert um 90° Grad, Grad wird zu 0° (außer System überlastet)
                // Schaltet LEDs in Abhängigkeit von vorher leuchtenden LEDs an.
                switch (lastMode) {
                    case ALL:
                        *mode = GREEN90;
                        lastMode = GREEN90;
                        digitalWrite(P_GREEN, HIGH);
                        break;
                    case GREEN90:
                        *mode = GREEN_YELLOW;
                        lastMode = GREEN_YELLOW;
                        digitalWrite(P_GREEN, HIGH);
                        digitalWrite(P_YELLOW, HIGH);
                        break;
                    case GREEN_YELLOW:
                        *mode = GREEN270;
                        lastMode = GREEN270;
                        digitalWrite(P_GREEN, HIGH);
                        break;
                    case GREEN270:
                        *mode = ALL;
                        lastMode = ALL;
                        digitalWrite(P_GREEN, HIGH);
                        digitalWrite(P_YELLOW, HIGH);
                        digitalWrite(P_RED, HIGH);
                        break;
                }
            }
        }
        usleep(delayTime);
    }
}

#pragma endregion LED

#pragma region Camera

/**
 * Nimmt ein Foto mit der piCam auf.
 *
 * @param filename der Dateiname, an dem das Foto gespeichert werden soll
 * @return true, wenn das Foto erfolgreich geschossen und abgespeichert wurde, andernfalls false
 */
bool picam_photo(const char* filename){
    cv::VideoCapture cap;
    if(!cap.open(0, cv::CAP_ANY)) {
        return false; // Konnte Gerät nicht öffnen
    }
    cv::Mat frame;
    if(!cap.read(frame)){
        cap.release();
        return false; // Konnte den Frame nicht lesen
    }
    cap.release();
    if(!cv::imwrite(filename, frame)) {
        return false; // Konnte die Datei nicht schreiben
    }
    return true;
}

/**
 * Nimmt jedes mal ein Foto auf, wenn mindestens eine LED leuchtet.
 *
 * @param mode der Modus, welcher angibt, welche LEDs leuchten
 */
void camera(const LED_MODE *mode) {
    LED_MODE lastMode = ALL;
    while (true) {
        std::time_t result = std::time(nullptr);    // Systemzeit auslesen
        // Systemzeit LESBAR speichern  AUSGABE: WEEKDAY MONTH DAY HH:MM:SS YEAR\n
        std::string time = std::asctime(std::localtime(&result));
        time = time.substr(0, time.size() - 1); // Standardmäßiges \n löschen.

        std::lock_guard<std::mutex> ledLock(ledMutex);
        // Modus muss ungleich dem letzten Modus sein, da sonst ggf. mehrere Fotos direkt hintereinander geschossen werden.
        if ((*mode) != NONE && lastMode != *mode) {
            bool success = picam_photo(("photos/" + time + ".png").c_str());
            if (!success) {
                std::cerr << "Konnte kein Bild aufnehmen" << std::endl;
            }

            lastMode = *mode;
        }
    }
}

#pragma endregion Camera

int main() {
    double degree = 0;

    wiringPiSetup();

    for (int i : P_STEP_MOTOR) {
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);
    }
    pinMode(P_GREEN, OUTPUT);
    pinMode(P_YELLOW, OUTPUT);
    pinMode(P_RED, OUTPUT);

    resetLEDS(); // LEDS ausschalten vor Programmbeginn
    LED_MODE ledMode = NONE; // Setzt den LED Modus auf NONE (alle LEDs aus).

    if (!std::filesystem::exists("photos")) {
        std::filesystem::create_directory("photos");
    }

    std::thread thread1 = std::thread(stepMotor, &degree);
    std::thread thread2 = std::thread(camera, &ledMode);
    std::thread thread3 = std::thread(led, &degree, &ledMode);

    thread1.join();

    usleep(delayTime); // Delay, damit das letzte Foto noch abgespeichert werden kann.
    thread2.detach();
    thread3.detach();

    resetLEDS(); // LEDS ausschalten nach Programmende

    return 0;
}
