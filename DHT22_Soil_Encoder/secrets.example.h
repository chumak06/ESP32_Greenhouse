// Шаблон локальных данных сети.
// Скопируйте этот файл в secrets.h и впишите свои значения:
//   secrets.example.h  ->  secrets.h
// Файл secrets.h добавлен в .gitignore и на GitHub не попадает.
// ESP32 работает только с сетями 2.4 ГГц.
#pragma once

constexpr char WIFI_SSID[] = "MyNetwork";      // имя сети
constexpr char WIFI_PASS[] = "MyPassword";     // пароль
