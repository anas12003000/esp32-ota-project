# BLE Scanner Project Architecture and Diagrams

## System Architecture

```mermaid
graph TB
    subgraph Cloud Infrastructure
        MQTT[MQTT Broker<br/>mqttiot.loranet.my:1885]
        DB[(Database)]
    end

    subgraph ESP32 Device
        BLE[BLE Scanner]
        WIFI[WiFi Client]
        OTA[OTA Handler]
        Flash[Flash Memory]
    end

    subgraph Client Tools
        PIO[PlatformIO]
        Script[Python OTA Script]
    end

    %% Data Flow
    BLE -->|Scan Results| WIFI
    WIFI -->|Publish| MQTT
    MQTT -->|Subscribe| WIFI
    Script -->|Firmware| MQTT
    MQTT -->|Firmware Chunks| OTA
    OTA -->|Update| Flash
    PIO -->|Build| Script
```

## OTA Update Sequence

```mermaid
sequenceDiagram
    participant Dev as Developer
    participant Script as Python Script
    participant MQTT as MQTT Broker
    participant ESP as ESP32
    participant Flash as Flash Memory

    Dev->>Script: Build & Start Update
    Script->>MQTT: START:filesize
    MQTT->>ESP: START:filesize
    ESP->>Flash: Begin Update
    
    loop Firmware Transfer
        Script->>MQTT: Firmware Chunks
        MQTT->>ESP: Firmware Chunks
        ESP->>Flash: Write Chunk
        ESP->>MQTT: Progress Update
    end

    Script->>MQTT: END
    MQTT->>ESP: END
    ESP->>Flash: Finalize Update
    ESP->>ESP: Reboot
    ESP->>MQTT: Connection Restored
```

## Memory Partition Layout

```mermaid
graph LR
    subgraph ESP32 Flash Memory
        Boot[Boot<br/>0x1000]
        Partition[Partition Table<br/>0x8000]
        OTA0[OTA0/Factory<br/>0x10000]
        OTA1[OTA1<br/>0x110000]
        SPIFFS[SPIFFS<br/>0x210000]
    end

    style Boot fill:#f9f,stroke:#333,stroke-width:2px
    style OTA0 fill:#bbf,stroke:#333,stroke-width:2px
    style OTA1 fill:#bbf,stroke:#333,stroke-width:2px
    style SPIFFS fill:#bfb,stroke:#333,stroke-width:2px
```

## BLE Scanning Process

```mermaid
graph LR
    subgraph ESP32 Process Flow
        Start[Start Scan] -->|30s| Scan[BLE Scan Active]
        Scan -->|Found Device| Process[Process MAC]
        Process -->|New MAC| Publish[Publish to MQTT]
        Process -->|Known MAC| Filter[Filter Out]
        Scan -->|Timeout| Wait[Wait 5s]
        Wait -->|Restart| Start
    end

    style Start fill:#f96,stroke:#333,stroke-width:2px
    style Scan fill:#9cf,stroke:#333,stroke-width:2px
    style Process fill:#9fc,stroke:#333,stroke-width:2px
```

## Network Architecture

```mermaid
graph TB
    subgraph Internet
        MQTT[MQTT Broker]
        NTP[NTP Server]
    end

    subgraph Local Network
        Router[WiFi Router]
        ESP32[ESP32 Device]
        Devices[BLE Devices]
    end

    subgraph Development
        PC[Development PC]
        VSCode[VS Code + PlatformIO]
    end

    ESP32 -->|WiFi| Router
    Router -->|Internet| MQTT
    Router -->|Internet| NTP
    ESP32 -->|Scan| Devices
    PC -->|Upload| ESP32
    VSCode -->|Build| PC
    PC -->|OTA Update| MQTT

    style ESP32 fill:#bbf,stroke:#333,stroke-width:2px
    style MQTT fill:#fbb,stroke:#333,stroke-width:2px
    style Devices fill:#bfb,stroke:#333,stroke-width:2px
```

## Project Directory Structure

```mermaid
graph TB
    Root[ble-scanner] -->|Source Code| SRC[src/]
    Root -->|Configuration| PIO[platformio.ini]
    Root -->|Documentation| DOC[Documentation/]
    Root -->|Tools| TOOLS[tools/]
    SRC -->|Main Code| MAIN[main.cpp]
    DOC -->|OTA Guide| OTA[OTA_MQTT_GUIDE.md]
    DOC -->|Architecture| ARCH[PROJECT_ARCHITECTURE.md]
    TOOLS -->|OTA Script| OTAPY[ota_updater.py]
    TOOLS -->|Dependencies| REQ[requirements.txt]
```
