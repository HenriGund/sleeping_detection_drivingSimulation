const int BUZZ_PIN = 9;
bool buzzing = false;

void setup() {
  pinMode(BUZZ_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "BUZZ_ON") buzzing = true;
    if (cmd == "BUZZ_OFF") buzzing = false;
  }

  if (buzzing) {
    tone(BUZZ_PIN, 2000);
  } else {
    noTone(BUZZ_PIN);
  }

  delay(10);
}
