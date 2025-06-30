# BrisaPUC

This repository contains all the code and files related to the BRISA project - A smart telemetry buoy.

---

## Requirements for MongoDB

### Python Dependencies
First, install all the required Python dependencies:

```bash
pip install pyserial websockets aiohttp motor pymongo
```

### MongoDB Installation
Follow these steps to install MongoDB on Ubuntu 24.04. These instructions are based on [this guide](https://www.cherryservers.com/blog/install-mongodb-ubuntu-2404):

1. Update and upgrade your system:
    ```bash
    sudo apt update
    sudo apt upgrade
    ```

2. Install required tools:
    ```bash
    sudo apt install -y gnupg curl
    ```

3. Add the MongoDB GPG key:
    ```bash
    curl -fsSL https://www.mongodb.org/static/pgp/server-8.0.asc | sudo gpg -o /usr/share/keyrings/mongodb-server-8.0.gpg --dearmor
    ```

4. Add the MongoDB repository:
    ```bash
    echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-8.0.gpg ] https://repo.mongodb.org/apt/ubuntu noble/mongodb-org/8.0 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-8.0.list
    ```

5. Update the package list and install MongoDB:
    ```bash
    sudo apt update
    sudo apt install -y mongodb-org
    ```

6. Start and enable the MongoDB service:
    ```bash
    sudo systemctl start mongod
    sudo systemctl enable mongod
    ```

---

## Check Database Information

To check the database information, follow these steps:

1. Open a terminal and switch to the desired database:
    ```bash
    mongosh telemetria_boya
    ```

2. Display the last 10 records sorted by timestamp:
    ```bash
    db.datos_sensores.find().sort({timestamp: -1}).limit(10).pretty()
    ```
