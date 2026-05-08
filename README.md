# ESP32-CAM AI Thinker

Проект содержит четыре отдельные прошивки PlatformIO для AI Thinker ESP32-CAM с
камерой OV2640:

- `diagnostic`: диагностика платы, камеры и PSRAM через Serial, без Wi-Fi.
- `web_photo`: Wi-Fi веб-интерфейс с live-view через JPEG polling, настройками
  камеры, сохранением настроек и сбросом к defaults.
- `mosaic_reader`: Wi-Fi интерфейс настройки для распознавания 4x3 мозаики.
  ESP32 берет raw RGB565 кадр, стабилизирует автоэкспозицию warm-up кадрами,
  семплит 12 точек по настраиваемой сетке и сам определяет `yellow`, `green`,
  `blue` или `white` по нормализованному цвету.
- `mosaic_reader_v2`: single-shot detector для робота. ESP32 сам ищет мозаику
  на одном RGB565 кадре и возвращает 12 цветов в режиме best-effort.

English version: [README.en.md](README.en.md)

![Web UI](docs/web-ui.png)

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

Собрать прошивку распознавания мозаики:

```powershell
.\esp32cam.cmd build -Environment mosaic_reader
```

Залить прошивку распознавания мозаики:

```powershell
.\esp32cam.cmd upload -Port COM7 -Environment mosaic_reader
```

Собрать v2 single-shot detector:

```powershell
.\esp32cam.cmd build -Environment mosaic_reader_v2
```

Залить v2 single-shot detector:

```powershell
.\esp32cam.cmd upload -Port COM7 -Environment mosaic_reader_v2
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

## Mosaic Reader

![Mosaic Reader UI](docs/mosaic-reader.png)

Скриншот показывает setup UI основной прошивки: слева raw RGB565 кадр с
настроенной сеткой, справа результат 3x4, confidence, warm-up и калибровка
цветов. В UI двигаются углы сетки `1`, `4`, `9`, `12`; остальные 8 точек
рассчитываются автоматически как ровная 4x3 сетка.

`mosaic_reader` подключается к Wi-Fi и поднимает setup UI на порту `80`.
Перед сборкой скопируй пример секретов:

```powershell
Copy-Item src\mosaic_reader\wifi_secrets.example.h src\mosaic_reader\wifi_secrets.h
```

Затем заполни `WIFI_SSID` и `WIFI_PASSWORD` в
`src\mosaic_reader\wifi_secrets.h`. Файл игнорируется git.

Endpoints:

- `GET /`: setup UI с raw RGB565 preview, красной сеткой, угловыми handles,
  калибровкой, warm-up настройкой и таблицей результата 3x4.
- `GET /frame?res=qqvga|qvga&radius=0..10&warmup=0..8`: один RGB565 кадр;
  ESP32 выбрасывает warm-up/stale кадры, распознает 12 точек и возвращает raw
  bytes плюс headers с width/height/result.
- `GET /result?res=qqvga|qvga&radius=0..10&warmup=0..8`: захват кадра без
  картинки, только JSON результата.
- `GET /status`: IP, камера, PSRAM, resolution, radius, warm-up, counters,
  точки, calibration status и last result.
- `POST /points`: сохранить 12 нормализованных координат точек в NVS. UI обычно
  отправляет сетку, рассчитанную из 4 углов.
- `POST /calibrate?point=0..11&color=yellow|green|blue|white`: взять sample из
  выбранной точки и сохранить калибровку цвета.
- `POST /settings/reset`: сбросить точки, radius, warm-up, resolution и
  calibration.

Распознавание:

- sample берется как небольшая область вокруг точки; почти черные пиксели
  рамки игнорируются, чтобы попадание на край ячейки не портило средний цвет;
- классификация идет по нормализованным долям `R/(R+G+B)`, `G/(R+G+B)`,
  `B/(R+G+B)`, поэтому результат меньше зависит от общей яркости;
- `Warm-up frames` по умолчанию `4`: это помогает AWB/AEC/AGC камеры
  стабилизироваться перед рабочим кадром.

Браузер не распознает цвета. Он только показывает raw кадр и отправляет
перемещения/калибровку; расчет остается на ESP32, чтобы позже тот же результат
можно было отдать через I2C.

## Mosaic Reader v2

`mosaic_reader_v2` рассчитан на роботный сценарий: один запрос делает один
кадр, ищет мозаику заново и возвращает 12 цветов. Tracking и маркеры не
используются.

Перед сборкой для своей сети:

```powershell
Copy-Item src\mosaic_reader_v2\wifi_secrets.example.h src\mosaic_reader_v2\wifi_secrets.h
```

Endpoints:

- `GET /`: debug/setup UI с raw RGB565 кадром, найденной сеткой, 4 начальными
  углами модели и таблицей результата.
- `GET /frame`: один RGB565 кадр плюс `X-Mosaic-Result` header для UI.
- `GET /result`: runtime JSON для робота: `status`, `found`, `confidence`,
  `pattern`, `corners`, `points`.
- `POST /model`: сохранить 4 начальных угла области поиска.
- `POST /calibrate?cell=0..11&color=yellow|green|blue|white`: обновить
  calibration по выбранной ячейке.
- `POST /settings/reset`: сбросить модель и calibration.

Если detector не уверен, он все равно возвращает 12 цветов со
`status: "best_effort"` и низким `confidence`. HTTP 503 остается только для
реальных ошибок камеры/capture.

## Железо и ограничения

Проверялось на AI Thinker ESP32-CAM с OV2640 и USB-UART адаптером CH340.
Рабочий порт в этой сборке был `COM7`.

Важные ограничения:

- многие китайские ESP32-CAM выглядят как AI Thinker, но PSRAM может не работать
  или отсутствовать;
- без рабочей PSRAM лучше использовать `QQVGA` или `QVGA`;
- `VGA` доступен для проверки, но без PSRAM может падать с HTTP 503;
- web-стрим сделан через JPEG polling, чтобы настройки оставались отзывчивыми
  даже без PSRAM.

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
