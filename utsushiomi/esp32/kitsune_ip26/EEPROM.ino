
void EEPROM_write() {
  String ssid = "utsushiomi";
  EEPROM.writeString(address, ssid);
  address += ssid.length() + 1;

  String pass = "remokuma";
  EEPROM.writeString(address, pass);
  address += pass.length() + 1;

  String l_ip = "192.168.11.26";
  EEPROM.writeString(address, l_ip);
  address += l_ip.length() + 1;

  String g_ip = "192.168.11.1";
  EEPROM.writeString(address, g_ip);
  address += g_ip.length() + 1;

  String s_ip = "255.255.255.0";
  EEPROM.writeString(address, s_ip);
  address += s_ip.length() + 1;

  EEPROM.commit();
}

void EEPROM_read() {
  ssid = EEPROM.readString(address);
//  Serial.println(ssid);
  address += ssid.length() + 1;

  password = EEPROM.readString(address);
//  Serial.println(password);
  address += password.length() + 1;

  local_ip_str = EEPROM.readString(address);
//  Serial.println(local_ip_str);
  address += local_ip_str.length() + 1;

  gateway_ip_str = EEPROM.readString(address);
//  Serial.println(gateway_ip_str);
  address += gateway_ip_str.length() + 1;

  subnet_ip_str = EEPROM.readString(address);
//  Serial.println(subnet_ip_str);
  address += subnet_ip_str.length() + 1;

  EEPROM.commit();
}
