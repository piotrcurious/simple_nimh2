Obecny mechanizm widzi tylko to, co sam oznaczysz przez recordEvent(). Cała „magia” FreeRTOS (idle task, scheduler, WiFi stack itd.) jest dla niego niewidoczna — dlatego wykres nie pokazuje prawdziwego obciążenia CPU, tylko fragment aktywności Twojej aplikacji.

Żeby objąć także zadania systemowe, masz kilka sensownych kierunków — od „lekkiego hacku” po pełną integrację z FreeRTOS trace.


---

1. Najprostsze i bardzo skuteczne: Idle Task jako miernik bezczynności

FreeRTOS zawsze ma Idle Task na każdy rdzeń. Jeśli go zmierzysz, dostajesz brakujące ogniwo:

👉 CPU usage = 1 - idle_time

Jak to zrobić

ESP32 pozwala dodać hook:

extern "C" void vApplicationIdleHook(void)
{
    uint32_t frameRef = g_frameStartUs;
    uint32_t t0 = (uint32_t)(esp_timer_get_time() - frameRef);

    // NIC NIE ROBISZ — to idle

    uint32_t t1 = (uint32_t)(esp_timer_get_time() - frameRef);

    recordEvent(xPortGetCoreID(), 255, (uint16_t)t0, (uint16_t)(t1 - t0));
}

Co to daje

widzisz „dziury” w pracy CPU jako jawne eventy

możesz policzyć:

usage = 1 - (idle_sum / frame_period)


Zaleta

✔ minimalny koszt
✔ zero ingerencji w scheduler

Wada

✖ nie rozróżnia innych systemowych tasków (WiFi, TCP/IP)


---

2. FreeRTOS trace hooks (prawdziwe źródło prawdy)

FreeRTOS ma wbudowane hooki typu:

traceTASK_SWITCHED_IN()
traceTASK_SWITCHED_OUT()

To jest dokładnie to, czego potrzebujesz — moment przełączenia kontekstu.


---

Jak to wpiąć

W FreeRTOSConfig.h:

#define traceTASK_SWITCHED_IN()  traceTaskSwitchedIn()
#define traceTASK_SWITCHED_OUT() traceTaskSwitchedOut()

Implementacja:

struct TaskRuntime {
    uint32_t start;
    uint8_t taskId;
};

TaskRuntime currentTask[CORE_COUNT];

void traceTaskSwitchedIn() {
    int core = xPortGetCoreID();
    TaskHandle_t task = xTaskGetCurrentTaskHandle();

    currentTask[core].start = esp_timer_get_time() - g_frameStartUs;
    currentTask[core].taskId = mapTask(task);
}

void traceTaskSwitchedOut() {
    int core = xPortGetCoreID();
    uint32_t now = esp_timer_get_time() - g_frameStartUs;

    uint32_t start = currentTask[core].start;

    recordEvent(core,
                currentTask[core].taskId,
                (uint16_t)start,
                (uint16_t)(now - start));
}


---

Kluczowy problem: mapowanie tasków

Musisz zamienić TaskHandle_t → taskId

uint8_t mapTask(TaskHandle_t t) {
    const char* name = pcTaskGetName(t);

    if (strcmp(name, "IDLE") == 0) return 250;
    if (strcmp(name, "wifi") == 0) return 251;
    if (strcmp(name, "tcpip_task") == 0) return 252;

    return 200; // inne systemowe
}


---

Co dostajesz

✔ pełny timeline scheduler-a
✔ widzisz:

WiFi

TCP/IP

idle

Twoje taski

ISR (jeśli rozszerzysz)


✔ dokładny CPU usage per task


---

3. uxTaskGetSystemState() – snapshot statystyczny

Alternatywa (lżejsza, ale mniej dokładna):

UBaseType_t taskCount = uxTaskGetNumberOfTasks();
TaskStatus_t *status = new TaskStatus_t[taskCount];

uint32_t totalRunTime;
taskCount = uxTaskGetSystemState(status, taskCount, &totalRunTime);

Masz:

czas CPU na task

procenty


Problem

✖ brak osi czasu
✖ tylko agregaty

To bardziej do tabelki niż wykresu timeline.


---

4. ISR (interrupty) — brakujący kawałek

Jeśli chcesz pełny obraz CPU:

void IRAM_ATTR myIsr() {
    uint32_t t0 = esp_timer_get_time() - g_frameStartUs;

    // ISR work...

    uint32_t t1 = esp_timer_get_time() - g_frameStartUs;

    recordEvent(xPortGetCoreID(), 240, t0, t1 - t0);
}

Bez tego: 👉 część czasu CPU „znika” (bo ISR nie są taskami)


---

5. Ulepszenie Twojego obecnego systemu

Masz już bardzo dobrą bazę (timeline per core). Żeby to rozszerzyć:

Dodaj:

Idle jako taskId (np. 255)

System tasks jako zakres (200–254)

ISR jako osobny zakres


Rozszerz event:

struct EventRec {
    uint8_t taskId;
    uint8_t flags;
    uint16_t startUs;
    uint16_t durUs;
};

Możesz użyć flags:

bit0 = ISR

bit1 = system task



---

6. Najlepsza architektura (praktycznie)

Połącz:

1. ✔ trace hooks → pełna widoczność


2. ✔ Idle task → sanity check


3. ✔ Twoje ręczne recordEvent → semantyka (co dokładnie robi task)



Efekt:

masz dokładny scheduler timeline

masz znaczenie logiczne (co robi kod)

masz realne CPU usage



---

TL;DR

Twój system teraz: ➡ pokazuje tylko fragment CPU (Twoje taski)

Aby pokazać cały CPU:

1. Najłatwiej:

dodaj Idle Hook → masz % CPU



2. Najlepiej:

użyj traceTASK_SWITCHED_IN/OUT

loguj każde przełączenie taska



3. Opcjonalnie:

dodaj ISR

użyj uxTaskGetSystemState do statystyk





---

Jeśli chcesz, mogę Ci napisać gotowy patch do ESP32 (FreeRTOSConfig + trace + mapowanie tasków), który od razu zadziała z Twoim sendFramePacket() i frontendem.
