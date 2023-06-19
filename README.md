Команда Роботикс (Robotx). Описание робота для соревновний FutureEngineers 2023.

Файловая структура:

models - модели для 3d печати.

t-photos - фото команды.

v-photos - фото транспортного средства.

schemes - схема электронных компонентов.

src - исходный код программ.

Робот был сделан на основе самостоятельно разработанного и напечатанного на 3D-принтере шасси, а так же элементов LEGO. 3D модели для печати находятся в папке models. Вариант самостоятельной разработки шасси был выбран по двум причинам:

- возможность сделать шасси нужной конфигурации для оптимального расположения электронных и механических компонентов робота
- уменьшения массы робота и улучшенния маневренности по сравнению с готовыми радиоуправляемыми шасси
  
Система перемещения робота:

Автомобиль состоит из: двусоставной рамы(из напечатанных деталей и частей конструктора LEGO, крепления мотора, крепления камеры, крепления датчиков, дисков колес и силиконовых покрышек.(car_frame.stl, motor_mount.stl, camera_mount.stl, sensor_mount.stl, rim.stl, wheels1.stl, wheels2.stl)

Электронная составляющая робота: ESP32(главный микроконтроллер), OpenMV H7(второстепенный контроллер для обработки изображения с камеры), стабилизатор напряжения mini360, двигатель MR12-020, драйвер двигателя ta6586, инфракрасный дальномер OPT3101, сервопривод Tower Pro MG90S, а питание идет от аккамулятора LiPo 2s (7.4V) 460mAh.

Используемые датчики: Инфракрасный дальномер OPT3101 (для определения расстояния до стены), фоторезистор со светодиодной подсветкой (для определения цвета линий).

Подключение всех электронных эллементов происходит через провода в силиконовой оболочке т. к. пайка дает лучше соеденение чем стандартные коннекторы.

Для крепления компонентов много где используются пластиковые стойки и пластиковые винты с гайками.

Робот включается с помощью выключателя на кабеле к батарее. Запускаются оба контроллера. После этого ESP32 инициализирует драйвер/основной двигатель, сервопривод, датчик расстояния и датчик линии. После этого издается звуковой сигнал и робот ждет нажатия на кнопку старта.

Логика работы программы:

Квалификационный заезд (робот устанавливается на поле строго вдоль бортов, основная программа начинает работать после нажатия кнопки старта):

1. Робот двигается вперед строго прямо до нахождения синей/оранжевой полосы с помощью датчика яркости света
2. Если найдена синяя/оранжевая полоса, то в зависимости от цвета линии, робот едет с поворотом колес налево/направо на нужный угол необходимое количество времени таким образом, чтобы подъехать к перпендикулярной стене на расстояние примерно 20см и встать вдоль нее. Угол и время проезда подбирается опытным путем.
3. Затем запускается блок программы езды вдоль внешней стены полигона с помощью дальномеров и PID-регулятора. Робот едет на расстоянии 20см от внешней стены. Т.е. если первый поворот был налево и внешняя стенка справа, то едем по правому дальномеру. И наоборот.
4. Во время езды вдоль стены анализируем показания датчика яркости и определяем наезд на синюю/оранжевую линию.
5. Если наехали на синюю/оранжевую линию, то робот поворачивает колеса на нужный угол налево/направо и проезжает неободимое расстояние для того, чтобы подъехать к перпендикулярной стене на расстояние 20см и встать вдоль нее. Угол и расстояние проезда подбирается опытным путем.
Затем алгоритм повторяется с п.3 до тех пор пока количество поворотов не достигнет 12.
После проезда двенадцатого поворота проезжаем заранее подобранное время с заранее подобранным углом поворота колес и останавливаемся.

Финальный заезд (робот устанавливается на поле строго вдоль бортов, основная программа начинает работать после нажатия кнопки старта):

1. Двигается вперед строго прямо до нахождения синей/оранжевой полосы с помощью датчика яркости света если перед роботом стоит кубик, то объезжаем его.
2. Если найдена синяя/оранжевая полоса, то в зависимости от цвета линии, робот едет с поворотом колес налево/направо на нужный угол необходимое количество времени таким образом, чтобы подъехать к перпендикулярной стене на расстояние примерно 20см и встать вдоль нее. Угол и время проезда подбирается опытным путем.
3. Затем запускается блок программы езды вдоль внешней стены полигона с помощью дальномеров и PID-регулятора. Робот едет на расстоянии 20см от внешней стены. Т.е. если первый поворот был налево и внешняя стенка справа, то едем по правому дальномеру. И наоборот.
Во время езды вдоль стены анализируем показания с контроллера OpenMV H7. Если контроллер OpenMV находит препятствие, то передает данные о цвете и координатах препятствия на контроллер ESP32. ESP32 направляет робота на препятсвие (постоянно получая данные с OpenMV о координатах препятсвия) для того, чтобы выровнять робота относительно препятствия. Т.е. препятсвие будет находиться строго перед роботом. Далее робот делает маневр объезда препятсвия в зависимости от цвета слева или справа. Объезд (углы поворота колес и время проезда) настраивается опытным путем. После объезда робот снова начинает ехать вдоль внешней стены.
4. Одновременно с получением данных с OpenMV, контроллер ESP32 анализирует показания датчика яркости и определяет наезд на синюю/оранжевую линию.
5. Если наехали на синюю/оранжевую линию, то робот поворачивает колеса на нужный угол налево/направо и проезжает неободимое расстояние для того, чтобы подъехать к перпендикулярной стене на расстояние 20см и встать вдоль нее. Угол и расстояние проезда подбирается опытным путем.
Затем алгоритм повторяется с п.3 до тех пор пока количество поворотов не достигнет 12.
После проезда двенадцатого поворота проезжаем заранее подобранное время с заранее подобранным углом поворота колес и останавливаемся.

Программа для ESP32 написана на языке C++, для OpenMV - на языке python. Исходный код находится в папке src.

qual.cpp - программа для квалификационного заезда на ESP32

final.cpp - программа для финального заезда на ESP32

main.py - программа для контроллера OpenMV H7

Программы для ESP32 используют стандартные библиотеки из фреймворка PlatformIO, компилируются и загружаются в контроллер в среде Microsoft Visual Studio Code. Программа для OpenMV H7 загружается в контроллер с помощью стандартной среды разработки от контроллеров OpenMV. (Скачать можно по ссылке https://openmv.io/pages/download)

Программы для квалификационного и финального заездов во многом схожи и состоят из таких блоков:

Блок добавления библиотек, определения переменных и создание объектов для управления сервоприводом, основным двигателем и дальномерами.
Функция SetID(). Инициализация лазерных дальномеров. Указание их режима и параметров работы.
Функция start(). Определение положения робота относительно стенок полигона, проезд до первого поворота, определение направления поворота и непосредственно выполнение поворота.
Функция setup(). Настройка параметров serial порта для передачи данных с контроллера OpenMV. Ожидание нажатия кнопки старта и запуск функции start() после нажатия кнопки.
Функция loop(). Основной цикл программы. В ней реализована вся логика программы описанная выше.
В программе для контроллера OpenMV H7 следующие блоки:

Импорт библиотек и определение переменных
Определение порогов цветов в цветовой модели LAB для определения объектов красного и зеленого цветов.
Настройка serial порта и режима работы камеры
Блок определения красных объектов в необходимой области видимости (ROI) и передача цвета и х-координаты в контроллер ESP32 по serial порту.
Блок определения зеленых объектов в необходимой области видимости (ROI) и передача цвета и х-координаты в контроллер ESP32 по serial порту.
