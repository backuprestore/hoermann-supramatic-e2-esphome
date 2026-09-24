#include "garage_door.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstring>

namespace esphome {
namespace garage_door {

static const char *const TAG = "garage_door";

void GarageDoorComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Garage Door (Hörmann Bus, Baureihe 2/3)...");
  if (write_enable_ != nullptr) {
    write_enable_->turn_off();  // RS485-Treiber auf Empfang
  }
}

void GarageDoorComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Garage Door Component");
  ESP_LOGCONFIG(TAG, "  Eigene Bus-Adresse: 0x%02X", own_address_);
  ESP_LOGCONFIG(TAG, "  Angenommene Master-Adresse: 0x%02X", master_address_);
}

void GarageDoorComponent::loop() {
  while (available() > 0) {
    if (rx_len_ >= MAX_FRAME) {
      // Puffer voll, ohne dass wir ein gültiges Frame erkannt haben -> verwerfen
      ESP_LOGW(TAG, "RX-Puffer voll ohne gültiges Frame, verwerfe %d Bytes", rx_len_);
      rx_len_ = 0;
    }
    rx_buffer_[rx_len_++] = (uint8_t) read();
    ESP_LOGVV(TAG, "RX byte: 0x%02X", rx_buffer_[rx_len_ - 1]);
    last_byte_ms_ = millis();
    try_parse_frames();
  }

  // Wenn seit dem letzten Byte eine Weile nichts mehr kam, aber noch Reste im
  // Puffer liegen, die zu keinem gültigen Frame passten: verwerfen (Resync).
  if (rx_len_ > 0 && (millis() - last_byte_ms_) > 10) {
    rx_len_ = 0;
  }
}

// ---------------------------------------------------------------------------
// CRC8, Polynom 0x07, Startwert 0xF3, MSB-first, kein finales XOR.
// Gegen 5 reale Beispiel-Telegramme aus der Hörmann-Bus-Doku verifiziert.
// ---------------------------------------------------------------------------
uint8_t GarageDoorComponent::calc_crc8(const uint8_t *data, uint8_t length) {
  uint8_t crc = 0xF3;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// Telegrammformat auf der Leitung:
// [Sync 0x00] [Zieladresse] [Zähler(4bit)|Länge(4bit)] [Nutzlast...] [CRC8]
// Das Sync-Byte gehört nicht zur CRC-Berechnung. Die Mitschnitte zeigen, dass
// jedes Telegramm mit diesem Byte beginnt; die Zieladresse kann ebenfalls
// 0x00 sein (Broadcast).
void GarageDoorComponent::try_parse_frames() {
  while (rx_len_ >= 1) {
    if (rx_buffer_[0] != FRAME_SYNC) {
      ESP_LOGVV(TAG, "Unerwartetes Byte 0x%02X, verwerfe zum Resync", rx_buffer_[0]);
      memmove(rx_buffer_, rx_buffer_ + 1, --rx_len_);
      continue;
    }
    if (rx_len_ < 3) {
      return;  // Sync, Adresse und Längenbyte noch nicht vollständig
    }

    uint8_t payload_len = rx_buffer_[2] & 0x0F;
    uint8_t total_len = 1 + 2 + payload_len + 1;  // Sync + Adresse + Header + Payload + CRC

    if (total_len > MAX_FRAME) {
      // Unplausible Länge -> vermutlich beschädigtes Längenbyte
      memmove(rx_buffer_, rx_buffer_ + 1, --rx_len_);
      continue;
    }
    if (rx_len_ < total_len) {
      return;  // noch nicht genug Bytes für dieses mögliche Frame
    }

    if (calc_crc8(rx_buffer_ + 1, total_len - 2) == rx_buffer_[total_len - 1]) {
      char hex[3 * MAX_FRAME + 1] = {0};
      for (uint8_t i = 0; i < total_len; i++) {
        snprintf(hex + i * 3, 4, "%02X ", rx_buffer_[i]);
      }
      ESP_LOGD(TAG, "Gültiges Frame: %s", hex);
      handle_frame(rx_buffer_ + 1, total_len - 1);
      uint8_t remaining = rx_len_ - total_len;
      memmove(rx_buffer_, rx_buffer_ + total_len, remaining);
      rx_len_ = remaining;
      // weiter versuchen, falls noch ein zweites Frame im Puffer steckt
    } else {
      // Sync bleibt ein gültiger Kandidat; verwerfe ihn erst bei CRC-Fehler.
      ESP_LOGVV(TAG, "CRC-Mismatch, verwerfe Sync-Byte zum Resync");
      memmove(rx_buffer_, rx_buffer_ + 1, --rx_len_);
    }
  }
}

void GarageDoorComponent::handle_frame(const uint8_t *frame, uint8_t total_len) {
  uint8_t dest = frame[0];
  uint8_t payload_len = frame[1] & 0x0F;
  const uint8_t *payload = frame + 2;

  // Der Antrieb sendet je nach Firmware entweder nur [status] oder
  // [0x01, status], wobei 0x01 der Broadcast-Befehl ist.
  // Rein passives Mitlesen, keine eigene Bus-Teilnahme nötig.
  if (dest == 0x00 && (payload_len == 1 || (payload_len >= 2 && payload[0] == 0x01))) {
    handle_broadcast_status(payload, payload_len);
    return;
  }

  // Ab hier: nur relevant, wenn WIR gemeint sind.
  if (dest != own_address_ || payload_len < 1) {
    ESP_LOGVV(TAG, "Frame nicht für uns (Ziel 0x%02X, unsere Adresse 0x%02X)", dest, own_address_);
    return;
  }

  if (payload[0] == 0x01) {
    // Bus-Scan: der Master fragt uns nach unserem Gerätetyp.
    handle_bus_scan(payload, payload_len);
  } else if (payload[0] == 0x20) {
    // Statusanfrage: der Master will wissen, ob wir eine Aktion auslösen wollen.
    handle_status_request();
  }
}

void GarageDoorComponent::handle_broadcast_status(const uint8_t *payload, uint8_t len) {
  uint8_t status = (len == 1) ? payload[0] : payload[1];

  if (status & hoermann_broadcast_bit::ERROR) {
    commanded_motion_ = hoermann_state_unknown;
    set_state(hoermann_state_error, "error");
  } else if (status & hoermann_broadcast_bit::MOVING_OPEN) {
    commanded_motion_ = hoermann_state_opening;
    set_state(hoermann_state_opening, "opening");
  } else if (status & hoermann_broadcast_bit::MOVING_CLOSE) {
    commanded_motion_ = hoermann_state_closing;
    set_state(hoermann_state_closing, "closing");
  } else if (status & hoermann_broadcast_bit::END_OPEN) {
    commanded_motion_ = hoermann_state_unknown;
    last_endpoint_ = hoermann_state_open;
    set_state(hoermann_state_open, "open");
  } else if (status & hoermann_broadcast_bit::END_CLOSED) {
    commanded_motion_ = hoermann_state_unknown;
    last_endpoint_ = hoermann_state_closed;
    set_state(hoermann_state_closed, "closed");
  } else {
    if (commanded_motion_ == hoermann_state_opening) {
      set_state(hoermann_state_opening, "opening");
    } else if (commanded_motion_ == hoermann_state_closing) {
      set_state(hoermann_state_closing, "closing");
    } else if (last_endpoint_ == hoermann_state_closed) {
      commanded_motion_ = hoermann_state_opening;
      set_state(hoermann_state_opening, "opening");
    } else if (last_endpoint_ == hoermann_state_open) {
      commanded_motion_ = hoermann_state_closing;
      set_state(hoermann_state_closing, "closing");
    } else {
      set_state(hoermann_state_stopped, "stopped");
    }
  }
}

void GarageDoorComponent::handle_bus_scan(const uint8_t *payload, uint8_t len) {
  // Beispiel aus der Doku: 0x28 0x02 0x01 0x80 0x0D
  //   payload = [0x01, 0x80] -> 0x80 ist die Absenderadresse (der Master)
  if (len >= 2) {
    master_address_ = payload[1];
  }
  // Antwort: [Gerätetyp, eigene Adresse]. 0x14 ist der Typcode, den das echte
  // UAP1 in Bounis Mitschnitt gemeldet hat.
  uint8_t reply[2] = {0x14, own_address_};
  send_frame(master_address_, reply, 2);
}

void GarageDoorComponent::handle_status_request() {
  uint8_t status_byte = 0x00;
  uint8_t status_flags = 0x10;

  if (pending_action_ != hoermann_action_none && pending_action_ticks_ > 0) {
    switch (pending_action_) {
      case hoermann_action_open:
        status_byte |= hoermann_impulse_bit::OPEN;
        break;
      case hoermann_action_close:
        status_byte |= hoermann_impulse_bit::CLOSE;
        break;
      case hoermann_action_venting:
        status_byte |= hoermann_impulse_bit::HALF_OPEN;  // UNSICHER, bitte verifizieren
        break;
      case hoermann_action_toggle_light:
        status_byte |= hoermann_impulse_bit::LIGHT_OUT;  // UNSICHER: innen/außen?
        break;
      case hoermann_action_stop:
        // Der Hörmann-Impuls 0x1004 stoppt die laufende Fahrt. Außerhalb
        // einer Fahrt kann derselbe Impuls als normaler Folgeimpuls wirken.
        status_byte |= hoermann_impulse_bit::FOLLOW_UP;
        break;
      case hoermann_action_impulse:
        status_byte |= hoermann_impulse_bit::FOLLOW_UP;
        break;
      case hoermann_action_emergency_stop:
        status_flags = 0x00;
        break;
      default:
        break;
    }
    pending_action_ticks_--;
    if (pending_action_ticks_ == 0) {
      pending_action_ = hoermann_action_none;
    }
  }

  // Die UAP1-Statusantwort enthält zwei Statusbytes. Das zweite Byte muss bei
  // Befehlen 0x10 enthalten; erst dadurch entsteht z.B. OPEN = 0x1001 bzw.
  // CLOSE = 0x1002. Ohne dieses Byte werden die Befehle vom Antrieb ignoriert.
  uint8_t reply[3] = {0x29, status_byte, status_flags};
  send_frame(master_address_, reply, 3);
}

void GarageDoorComponent::send_frame(uint8_t dest_addr, const uint8_t *payload, uint8_t payload_len) {
  uint8_t frame[MAX_FRAME];
  frame[0] = FRAME_SYNC;
  frame[1] = dest_addr;
  frame[2] = (uint8_t)(((tx_counter_ & 0x0F) << 4) | (payload_len & 0x0F));
  tx_counter_ = (tx_counter_ + 1) & 0x0F;
  memcpy(frame + 3, payload, payload_len);

  uint8_t len_before_crc = (uint8_t)(3 + payload_len);
  frame[len_before_crc] = calc_crc8(frame + 1, (uint8_t)(len_before_crc - 1));
  uint8_t total_len = (uint8_t)(len_before_crc + 1);

  char hex[3 * MAX_FRAME + 1] = {0};
  for (uint8_t i = 0; i < total_len; i++) {
    snprintf(hex + i * 3, 4, "%02X ", frame[i]);
  }
  ESP_LOGD(TAG, "TX Frame: %s", hex);

  if (write_enable_ != nullptr) {
    write_enable_->turn_on();
  }
  write_array(frame, total_len);
  flush();
  if (write_enable_ != nullptr) {
    write_enable_->turn_off();
  }
}

void GarageDoorComponent::set_state(hoermann_state_t state, const std::string &name) {
  actual_state_ = state;
  actual_state_string_ = name;
  if (state != last_published_state_) {
    ESP_LOGI(TAG, "Tor-Status geändert: %s", name.c_str());
    if (state_sensor_ != nullptr) {
      state_sensor_->publish_state(name);
    }
    last_published_state_ = state;
  }
  // Cover unabhängig vom "geändert"-Check aktualisieren, damit z.B. eine
  // laufende Öffnungsbewegung nicht nur einmal, sondern kontinuierlich als
  // "opening" gemeldet wird, solange der Broadcast das so meldet.
  if (cover_ != nullptr) {
    cover_->update_from_state(state);
  }
}

// ---------------------------------------------------------------------------
// GarageDoorCover: bildet den Hörmann-Bus-Status auf ein echtes ESPHome-
// Cover-Entity ab (statt eines "optimistic" Template-Covers zu raten).
// Da uns per Bus nur Endlagen + Bewegungsrichtung vorliegen (keine exakte
// Zwischenposition), wird supports_position bewusst NICHT gesetzt - Home
// Assistant zeigt dann ein einfaches Auf/Zu/Stop-Garagentor.
// ---------------------------------------------------------------------------
void GarageDoorCover::setup() {
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    this->position = 0.5f;  // Zustand unbekannt, bis der erste Broadcast eintrifft
  }
  this->current_operation = cover::COVER_OPERATION_IDLE;
}

void GarageDoorCover::dump_config() { LOG_COVER("", "Garage Door Cover", this); }

cover::CoverTraits GarageDoorCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_stop(true);
  traits.set_supports_position(false);  // nur Endlagen bekannt, keine echte Positionsmessung
  // Bei einem Zwischenstopp kennen wir keine exakte Position. Dadurch sollen
  // OPEN und CLOSE in Home Assistant immer verfügbar bleiben.
  traits.set_is_assumed_state(true);
  return traits;
}

void GarageDoorCover::control(const cover::CoverCall &call) {
  if (parent_ == nullptr) return;

  if (call.get_stop()) {
    parent_->action_stop();
  } else if (call.get_position().has_value()) {
    float pos = *call.get_position();
    if (pos == cover::COVER_OPEN) {
      parent_->action_open();
    } else if (pos == cover::COVER_CLOSED) {
      parent_->action_close();
    }
  }
}

void GarageDoorCover::update_from_state(hoermann_state_t state) {
  switch (state) {
    case hoermann_state_open:
      this->position = cover::COVER_OPEN;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      break;
    case hoermann_state_closed:
      this->position = cover::COVER_CLOSED;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      break;
    case hoermann_state_opening:
      this->current_operation = cover::COVER_OPERATION_OPENING;
      break;
    case hoermann_state_closing:
      this->current_operation = cover::COVER_OPERATION_CLOSING;
      break;
    case hoermann_state_stopped:
    case hoermann_state_venting:
    case hoermann_state_error:
    default:
      // Position bleibt unverändert (irgendwo dazwischen stehengeblieben) -
      // wir wollen NICHT fälschlich "offen" oder "zu" anzeigen.
      this->current_operation = cover::COVER_OPERATION_IDLE;
      break;
  }
  this->publish_state();
}

// Für OPEN/CLOSE reichen mehrere Polls als Schutz gegen verpasste Antworten.
static constexpr uint8_t IMPULSE_HOLD_TICKS = 3;
// STOP entspricht im Referenzimplementierung einem einzelnen 0x1004-Impuls.
static constexpr uint8_t STOP_IMPULSE_HOLD_TICKS = 1;

void GarageDoorComponent::action_open() {
  ESP_LOGD(TAG, "action_open called");
  commanded_motion_ = hoermann_state_opening;
  pending_action_ = hoermann_action_open;
  pending_action_ticks_ = IMPULSE_HOLD_TICKS;
}

void GarageDoorComponent::action_close() {
  ESP_LOGD(TAG, "action_close called");
  commanded_motion_ = hoermann_state_closing;
  pending_action_ = hoermann_action_close;
  pending_action_ticks_ = IMPULSE_HOLD_TICKS;
}

void GarageDoorComponent::action_stop() {
  ESP_LOGD(TAG, "action_stop called");
  const bool was_opening = actual_state_ == hoermann_state_opening || commanded_motion_ == hoermann_state_opening;
  const bool was_closing = actual_state_ == hoermann_state_closing || commanded_motion_ == hoermann_state_closing;
  const bool was_moving = was_opening || was_closing;
  commanded_motion_ = hoermann_state_unknown;
  last_endpoint_ = hoermann_state_unknown;
  if (!was_moving) {
    pending_action_ = hoermann_action_none;
    pending_action_ticks_ = 0;
    ESP_LOGD(TAG, "Ignoring stop while door is not moving");
    return;
  }
  // Dieses Gerät hält bei einem entgegengesetzten Fahrimpuls an: CLOSE
  // während OPEN stoppt die Aufwärtsfahrt und umgekehrt. Der separate
  // Folgeimpuls 0x1004 bleibt für den Impulse-Button verfügbar.
  if (was_opening) {
    pending_action_ = hoermann_action_close;
  } else {
    pending_action_ = hoermann_action_open;
  }
  pending_action_ticks_ = STOP_IMPULSE_HOLD_TICKS;
}

void GarageDoorComponent::action_venting() {
  ESP_LOGD(TAG, "action_venting called");
  pending_action_ = hoermann_action_venting;
  pending_action_ticks_ = IMPULSE_HOLD_TICKS;
}

void GarageDoorComponent::action_toggle_light() {
  ESP_LOGD(TAG, "action_toggle_light called");
  pending_action_ = hoermann_action_toggle_light;
  pending_action_ticks_ = IMPULSE_HOLD_TICKS;
}

void GarageDoorComponent::action_emergency_stop() {
  ESP_LOGW(TAG, "action_emergency_stop called");
  commanded_motion_ = hoermann_state_unknown;
  last_endpoint_ = hoermann_state_unknown;
  pending_action_ = hoermann_action_emergency_stop;
  pending_action_ticks_ = STOP_IMPULSE_HOLD_TICKS;
}

void GarageDoorComponent::action_impulse() {
  ESP_LOGD(TAG, "action_impulse called");
  pending_action_ = hoermann_action_impulse;
  pending_action_ticks_ = STOP_IMPULSE_HOLD_TICKS;
}

void GarageDoorEmergencyStopButton::press_action() {
  if (this->parent_ != nullptr) {
    this->parent_->action_emergency_stop();
  }
}

void GarageDoorImpulseButton::press_action() {
  if (this->parent_ != nullptr) {
    this->parent_->action_impulse();
  }
}

void GarageDoorVentingSwitch::write_state(bool state) {
  if (state) {
    this->parent_->action_venting();
  } else {
    this->parent_->action_close();
  }
  this->publish_state(state);
}

void GarageDoorLightSwitch::write_state(bool state) {
  if (state) {
    this->parent_->action_toggle_light();
  }
  this->publish_state(state);
}

}  // namespace garage_door
}  // namespace esphome