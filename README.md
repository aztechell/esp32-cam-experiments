# ESP32-CAM AI Thinker

Проект содержит две отдельные прошивки PlatformIO для AI Thinker ESP32-CAM с
камерой OV2640:

- `diagnostic`: диагностика платы, камеры и PSRAM через Serial, без Wi-Fi.
- `web_photo`: Wi-Fi веб-интерфейс с live-view через JPEG polling, настройками
  камеры, сохранением настроек и сбросом к defaults.

English version: [README.en.md](README.en.md)

## Локальная среда

Нужны Python 3 и Git. Проверить их наличие:

```powershell
python --version
git --version
```

Создать локальное виртуальное окружение и поставить инструменты:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install platformio esptool
```

Проверить PlatformIO:

```powershell
.\.venv\Scripts\pio.exe --version
```

PlatformIO и esptool используются из локальной `.venv`. PlatformIO складывает
свои пакеты в локальную `.platformio`, поэтому проект не должен писать
зависимости в домашнюю директорию пользователя. `.venv`, `.platformio` и `.pio`
не коммитятся.

Собрать диагностическую прошивку:

```powershell
.\esp32cam.cmd build -Environment diagnostic
```

Показать доступные serial-порты:

```powershell
.\esp32cam.cmd ports
```

Проверить ESP32-порт без прошивки:

```powershell
.\esp32cam.cmd chip -Ports COM7
```

Залить диагностическую прошивку:

```powershell
.\esp32cam.cmd upload -Port COM7 -Environment diagnostic
```

Собрать web-прошивку:

```powershell
.\esp32cam.cmd build -Environment web_photo
```

Залить web-прошивку:

```powershell
.\esp32cam.cmd upload -Port COM7 -Environment web_photo
```

Открыть serial monitor:

```powershell
.\esp32cam.cmd monitor -Port COM7
```

## Web-прошивка

`web_photo` подключается к настроенной Wi-Fi сети, печатает IP в serial monitor
и поднимает HTTP-сервер на порту `80`.

Перед сборкой для своей сети скопируй пример секретов:

```powershell
Copy-Item src\web_photo\wifi_secrets.example.h src\web_photo\wifi_secrets.h
```

Затем заполни `WIFI_SSID` и `WIFI_PASSWORD` в
`src\web_photo\wifi_secrets.h`. Этот файл добавлен в `.gitignore` и не должен
попадать в репозиторий.

Endpoints:

- `GET /`: веб-интерфейс.
- `GET /frame?res=qqvga|qvga|vga&fps=1|2|5|8|10`: один JPEG-кадр для live-view.
- `GET` или `POST /capture?res=qqvga|qvga|vga`: совместимый alias для одного JPEG-кадра.
- `GET /status`: JSON со статусом IP, PSRAM, камеры, активного/сохраненного
  разрешения, счетчика кадров, сохраненных настроек и последней ошибки.
- `GET` или `POST /settings/reset`: сброс сохраненных настроек web-прошивки.

Интерфейс показывает один live-дисплей и панель настроек камеры: JPEG quality,
brightness, contrast, saturation, sharpness, white balance, exposure, gain,
mirror, flip, lens correction и warm-up frame discard. Live-view стартует с
`2 fps`; можно выбрать `1`, `5`, `8` или `10 fps` без перепрошивки.

Настройки `web_photo` сохраняются в ESP32 NVS/Preferences и восстанавливаются
после перезагрузки. UI сначала читает `/status`, применяет сохраненные значения
к controls и записывает NVS только если настройка реально изменилась.

Если PSRAM не работает, web-прошивка использует один frame buffer в DRAM.
Практичные режимы для такой платы обычно `QQVGA` и `QVGA`; `VGA` доступен в UI,
но может вернуть HTTP 503 при нехватке памяти.

## Bootloader Mode

Если upload не подключается, переведи плату в bootloader mode:

1. Подключи `IO0` к `GND` или зажми `BOOT`.
2. Нажми и отпусти `RST`.
3. Запусти upload-команду.
4. Отключи `IO0` от `GND` или отпусти `BOOT`.
5. Нажми `RST` еще раз, чтобы прошивка стартовала.

Успешная диагностика в serial monitor должна содержать:

```text
camera init ok
capture ok: <bytes> bytes
jpeg markers: ok
probe done
```

Диагностическая прошивка также печатает heartbeat каждые 2 секунды:

```text
heartbeat: <ms> ms, camera: ready, count: <n>
```

## Если Monitor Пустой

Сначала проверь реальный открываемый порт:

```powershell
.\esp32cam.cmd ports
```

Для этой платы рабочим портом был `COM7`.

Если upload пишет `Failed to connect to ESP32: No serial data received`:

1. Зажми `BOOT` или соедини `IO0` с `GND`.
2. Нажми и отпусти `RST`, пока `BOOT/IO0` активен.
3. Запусти `.\esp32cam.cmd upload -Port COM7 -Environment web_photo`.
4. Отпусти `BOOT` или убери `IO0-GND`.
5. Открой `.\esp32cam.cmd monitor -Port COM7`.
6. Нажми и отпусти `RST`.
