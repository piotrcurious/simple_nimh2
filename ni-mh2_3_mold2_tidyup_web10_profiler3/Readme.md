Ten system nie mierzy „procentu CPU” bezpośrednio. On buduje czasowy zapis zdarzeń dla każdej klatki i z tego rysuje na UI zajętość procesora.

W skrócie działa to tak:

1. masterTask() wyznacza nową klatkę pomiarową

bierze czas startu frameStart = esp_timer_get_time(),

zapisuje go do g_frameStartUs,

zwiększa licznik klatek g_frameSeq,

zeruje bufory zdarzeń dla obu rdzeni:

g_coreBuf[0].count = 0;
g_coreBuf[1].count = 0;

potem czeka FRAME_PERIOD_US / 1000 ms i wysyła pakiet.


To znaczy: cały system profilu działa w ramach kolejnych okien czasowych.


2. Każdy task sam zgłasza swój czas wykonania W taskach masz schemat:

uint32_t frameRef = g_frameStartUs;
uint32_t t0 = esp_timer_get_time() - frameRef;
... wykonanie pracy ...
uint32_t t1 = esp_timer_get_time() - frameRef;
recordEvent(core, taskId, t0, t1 - t0);

Czyli:

startUs = ile mikrosekund minęło od początku klatki,

durUs = jak długo task pracował,

taskId identyfikuje zadanie,

core mówi, na którym rdzeniu to się działo.



3. recordEvent() zapisuje wpis do bufora rdzenia Funkcja wkłada rekord do g_coreBuf[core]:

g_coreBuf[core].events[idx].taskId = taskId;
g_coreBuf[core].events[idx].startUs = startUs;
g_coreBuf[core].events[idx].durUs   = durUs;

Jest to chronione sekcją krytyczną portENTER_CRITICAL(&g_mux), żeby równoległe taski nie nadpisały sobie danych.


4. sendFramePacket() wysyła zrzut do przeglądarki Po zakończeniu klatki kod:

kopiuje dane z buforów do lokalnych tablic,

buduje binarny pakiet,

wysyła go po WebSocket do klientów.


W pakiecie są między innymi:

numer klatki g_frameSeq,

czas startu klatki g_frameStartUs,

długość okresu klatki FRAME_PERIOD_US,

liczba zdarzeń na core 0 i core 1,

lista zdarzeń (taskId, flags, startUs, durUs).



5. Interfejs WWW prawdopodobnie rysuje z tego pasek czasu Na podstawie startUs i durUs frontend może narysować:

kiedy task wystartował,

jak długo trwał,

na którym rdzeniu pracował,

które fragmenty czasu były zajęte.


To daje wizualizację typu „timeline” albo „Gantt chart” dla obu rdzeni.



Najważniejsza rzecz: to jest profilowanie per klatka, a nie ciągły licznik obciążenia CPU.
Czyli system nie mówi „CPU ma 43%”, tylko pokazuje: „w tym oknie czasowym task A pracował tyle, task B tyle, na core 0 i core 1”.

Kilka istotnych szczegółów z tego kodu:

taskId rozróżnia zadania:

1 = task_readSHT4x

2 = task_processAdcDma

3 = task_updateSystemData

4 = task_webServer

0 = loop()


core jest zapisany jawnie, więc UI może pokazać osobne tory dla każdego rdzenia.

uint16_t startUs i uint16_t durUs ograniczają zakres do ok. 65 ms. Jeśli zadanie trwa dłużej, wartości się utną/zawiną.

ws.getClients() i client.queueLen() < 4 chronią system przed zapchaniem kolejki WebSocketów, żeby profilowanie nie zaczęło samo powodować lagów.


W praktyce ten mechanizm odpowiada na pytanie: „co robił procesor w ostatnim oknie czasu i kiedy dokładnie to robił?” a nie: „jaki jest średni procent użycia CPU od startu systemu?”

Jeśli chcesz, mogę teraz rozpisać ten mechanizm jeszcze dokładniej, linia po linii, albo narysować Ci schemat przepływu danych od `recordEvent()` do WebSocket i UI.
