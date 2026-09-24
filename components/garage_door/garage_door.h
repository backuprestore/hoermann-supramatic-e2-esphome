#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/button/button.h"

// ---------------------------------------------------------------------------
// Hörmann-Bus (UAP1-Protokoll, Baureihe 2/3 – NICHT die Serie-4/"UAP1-HCP"-
// Modbus-Variante, die z.B. bei der E4 verwendet wird!).
//
// Quelle: https://blog.bouni.de/posts/2018/hoerrmann-uap1/ – ein von der
// Community reverse-engineertes Protokoll, keine offizielle Hörmann-Doku.
// CRC8-Parameter (Poly 0x07, Init 0xF3) wurden gegen 5 der dort mitgeschnittenen
// Beispiel-Telegramme verifiziert und passen exakt.
//
// UNSICHER / bitte am eigenen Bus mit einem Logic Analyzer / UART-Sniffer
// verifizieren, bevor produktiv geschaltet wird:
//   - Die genaue Bit-Zuordnung für "Lüftung" und "Licht" (innen/außen) in
//     hoermann_impulse_bit
//   - STOP nutzt den dokumentierten Folgeimpuls 0x1004; EMERGENCY STOP nutzt
//     die separate Antwort 0x0000.
//   - Ob own_address_ (Default 0x28, wie das UAP1 in Bounis Mitschnitt) mit
//     einem evtl. bereits vorhandenen echten UAP1 am selben Bus kollidiert
// ---------------------------------------------------------------------------

// Bits im Statusbyte, das der Antrieb als BROADCAST (Zieladresse 0x00) an alle
// Busteilnehmer sendet. Rein passiv auslesbar, keine eigene Bus-Anmeldung
// nötig -> zuverlässigste Quelle für den Türzustand.
namespace hoermann_broadcast_bit {
  constexpr uint8_t END_OPEN     = 0x01;  // Endlage Auf erreicht
  constexpr uint8_t END_CLOSED   = 0x02;  // Endlage Zu erreicht
  constexpr uint8_t OPTION_RELAY = 0x04;
  constexpr uint8_t LIGHT_RELAY  = 0x08;
  constexpr uint8_t ERROR        = 0x10;
  constexpr uint8_t MOVING_CLOSE = 0x20;  // fährt Richtung Zu
  constexpr uint8_t MOVING_OPEN  = 0x40;  // fährt Richtung Auf
  constexpr uint8_t ALARM        = 0x80;
}

// Impuls-Bits, die WIR als emulierter Slave (wie ein UAP1) in unserer
// Statusantwort setzen, um eine Aktion auszulösen. Nur die ersten beiden
// (OPEN/CLOSE) sind aus den Beispieldaten direkt belegbar.
namespace hoermann_impulse_bit {
  constexpr uint8_t OPEN      = 0x01;
  constexpr uint8_t CLOSE     = 0x02;
  constexpr uint8_t FOLLOW_UP = 0x04;  // Folgeimpuls, auch für den normalen STOP
  constexpr uint8_t WICKET    = 0x08;  // Schlupftür
  constexpr uint8_t VACATION  = 0x10;
  constexpr uint8_t LIGHT_IN  = 0x20;
  constexpr uint8_t LIGHT_OUT = 0x40;
  constexpr uint8_t HALF_OPEN = 0x80;  // vermutlich = Lüftungsposition
}

typedef enum {
  hoermann_state_stopped = 0,
  hoermann_state_open,
  hoermann_state_closed,
  hoermann_state_venting,
  hoermann_state_opening,
  hoermann_state_closing,
  hoermann_state_error,
  hoermann_state_unknown
} hoermann_state_t;

typedef enum {
  hoermann_action_stop = 0,
  hoermann_action_open,
  hoermann_action_close,
  hoermann_action_venting,
  hoermann_action_toggle_light,
  hoermann_action_emergency_stop,
  hoermann_action_impulse,
  hoermann_action_none
} hoermann_action_t;

namespace esphome {
namespace garage_door {

class GarageDoorVentingSwitch : public switch_::Switch {
 public:
  void set_parent(class GarageDoorComponent *parent) { parent_ = parent; }
 protected:
  void write_state(bool state) override;
  class GarageDoorComponent *parent_;
};

class GarageDoorLightSwitch : public switch_::Switch {
 public:
  void set_parent(class GarageDoorComponent *parent) { parent_ = parent; }
 protected:
  void write_state(bool state) override;
  class GarageDoorComponent *parent_;
};

class GarageDoorEmergencyStopButton : public button::Button {
 public:
  void set_parent(class GarageDoorComponent *parent) { parent_ = parent; }
 protected:
  void press_action() override;
  class GarageDoorComponent *parent_{nullptr};
};

class GarageDoorImpulseButton : public button::Button {
 public:
  void set_parent(class GarageDoorComponent *parent) { parent_ = parent; }
 protected:
  void press_action() override;
  class GarageDoorComponent *parent_{nullptr};
};

class GarageDoorCover : public cover::Cover, public Component {
 public:
  void set_parent(class GarageDoorComponent *parent) { parent_ = parent; }
  void setup() override;
  void dump_config() override;
  cover::CoverTraits get_traits() override;
  // Vom Hauptcomponent bei jeder Statusänderung aufgerufen (siehe set_state()).
  void update_from_state(hoermann_state_t state);

 protected:
  void control(const cover::CoverCall &call) override;
  class GarageDoorComponent *parent_{nullptr};
};

class GarageDoorComponent : public Component, public uart::UARTDevice {
 public:
  GarageDoorComponent() {}

  void set_write_enable(output::BinaryOutput *write_enable) { write_enable_ = write_enable; }
  void set_state_sensor(text_sensor::TextSensor *sensor) { state_sensor_ = sensor; }
  void set_venting_switch(GarageDoorVentingSwitch *sw) { venting_switch_ = sw; }
  void set_light_switch(GarageDoorLightSwitch *sw) { light_switch_ = sw; }
  void set_own_address(uint8_t address) { own_address_ = address; }
  void set_cover(GarageDoorCover *cover) { cover_ = cover; }

  void setup() override;
  void loop() override;
  void dump_config() override;

  void action_open();
  void action_close();
  void action_stop();
  void action_venting();
  void action_toggle_light();
  void action_emergency_stop();
  void action_impulse();

  hoermann_state_t get_state() const { return actual_state_; }

 protected:
  output::BinaryOutput *write_enable_{nullptr};
  text_sensor::TextSensor *state_sensor_{nullptr};
  GarageDoorVentingSwitch *venting_switch_{nullptr};
  GarageDoorLightSwitch *light_switch_{nullptr};
  GarageDoorCover *cover_{nullptr};

  // Eigene emulierte Slave-Adresse. Default 0x28, wie das echte UAP1 in
  // Bounis Mitschnitt (liegt im Bereich 16-45 "intelligente Bedienteile").
  // Bei Adresskonflikt (z.B. zweites UAP1 am Bus) über set_own_address() ändern.
  uint8_t own_address_{0x21};

  // Adresse des Antriebs (Bus-Master). Default 0x80 laut Adresstabelle
  // ("128 = Master drive"), wird beim Bus-Scan zusätzlich aktualisiert.
  uint8_t master_address_{0x80};

  hoermann_state_t actual_state_{hoermann_state_unknown};
  hoermann_state_t commanded_motion_{hoermann_state_unknown};
  hoermann_state_t last_endpoint_{hoermann_state_unknown};
  std::string actual_state_string_{"unknown"};
  hoermann_state_t last_published_state_{hoermann_state_unknown};

  hoermann_action_t pending_action_{hoermann_action_none};
  uint8_t pending_action_ticks_{0};  // Anzahl Poll-Zyklen, über die der Impuls gehalten wird

  static const uint8_t MAX_FRAME = 16;
  static const uint8_t FRAME_SYNC = 0x00;
  uint8_t rx_buffer_[MAX_FRAME];
  uint8_t rx_len_{0};
  uint32_t last_byte_ms_{0};
  uint8_t tx_counter_{0};

  uint8_t calc_crc8(const uint8_t *data, uint8_t length);
  void try_parse_frames();
  void handle_frame(const uint8_t *frame, uint8_t total_len);
  void handle_broadcast_status(const uint8_t *payload, uint8_t len);
  void handle_bus_scan(const uint8_t *payload, uint8_t len);
  void handle_status_request();
  void send_frame(uint8_t dest_addr, const uint8_t *payload, uint8_t payload_len);
  void set_state(hoermann_state_t state, const std::string &name);
};

}  // namespace garage_door
}  // namespace esphome
