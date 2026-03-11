/**
 * STServo JavaScript SDK
 * Ported from official STServo Python SDK
 * For use with Web Serial API (Betaflight Style)
 */

export const STServoDef = {
    BROADCAST_ID: 0xFE,
    MAX_ID: 0xFC,
    
    // Instructions
    INST_PING: 1,
    INST_READ: 2,
    INST_WRITE: 3,
    INST_REG_WRITE: 4,
    INST_ACTION: 5,
    INST_SYNC_WRITE: 131,
    INST_SYNC_READ: 130,

    // Communication Results
    COMM_SUCCESS: 0,
    COMM_PORT_BUSY: -1,
    COMM_TX_FAIL: -2,
    COMM_RX_FAIL: -3,
    COMM_TX_ERROR: -4,
    COMM_RX_WAITING: -5,
    COMM_RX_TIMEOUT: -6,
    COMM_RX_CORRUPT: -7,
    COMM_NOT_AVAILABLE: -9
};

export const STSAddress = {
    // EPROM (Read-only)
    MODEL_L: 3,
    MODEL_H: 4,

    // EPROM (Read/Write)
    ID: 5,
    BAUD_RATE: 6,
    MIN_ANGLE_LIMIT_L: 9,
    MAX_ANGLE_LIMIT_L: 11,
    CW_DEAD: 26,
    CCW_DEAD: 27,
    OFS_L: 31,
    MODE: 33,

    // SRAM (Read/Write)
    TORQUE_ENABLE: 40,
    ACC: 41,
    GOAL_POSITION_L: 42,
    GOAL_TIME_L: 44,
    GOAL_SPEED_L: 46,
    LOCK: 55,

    // SRAM (Read-only)
    PRESENT_POSITION_L: 56,
    PRESENT_SPEED_L: 58,
    PRESENT_LOAD_L: 60,
    PRESENT_VOLTAGE: 62,
    PRESENT_TEMPERATURE: 63,
    MOVING: 66,
    PRESENT_CURRENT_L: 69
};

export class STServoSDK {
    constructor() {
        this.port = null;
        this.reader = null;
        this.writer = null;
        this.isOpening = false;
    }

    /**
     * Connect to a serial port via the Web Serial API
     */
    async connect(baudRate = 115200) {
        if (this.isOpening) return;
        this.isOpening = true;
        try {
            this.port = await navigator.serial.requestPort();
            await this.port.open({ baudRate });
            this.writer = this.port.writable.getWriter();
            console.log("STServo: Connected to port at", baudRate);
            return true;
        } catch (err) {
            console.error("STServo Connection Error:", err);
            return false;
        } finally {
            this.isOpening = false;
        }
    }

    async disconnect() {
        if (this.reader) await this.reader.cancel();
        if (this.writer) this.writer.releaseLock();
        if (this.port) await this.port.close();
        this.port = null;
    }

    /**
     * Internal: Calculate checksum
     */
    _calculateChecksum(packet) {
        let checksum = 0;
        // Checksum is calculated from ID to last parameter
        for (let i = 2; i < packet.length; i++) {
            checksum += packet[i];
        }
        return (~checksum) & 0xFF;
    }

    /**
     * Internal: Utility byte helpers
     */
    _loByte(w) { return w & 0xFF; }
    _hiByte(w) { return (w >> 8) & 0xFF; }
    _makeWord(l, h) { return (l & 0xFF) | ((h & 0xFF) << 8); }

    /**
     * Send raw instruction packet
     */
    async sendPacket(id, instruction, params = []) {
        if (!this.writer) throw new Error("Not connected");
        
        const length = params.length + 2; // Instruction + Params + Checksum
        const packet = [0xFF, 0xFF, id, length, instruction, ...params];
        packet.push(this._calculateChecksum(packet));
        
        const data = new Uint8Array(packet);
        await this.writer.write(data);
        return true;
    }

    /**
     * Read response packet from servo (with timeout)
     */
    async receivePacket(id, expectedLen, timeoutMs = 100) {
        if (!this.port) return null;
        
        const startTime = Date.now();
        let buffer = [];
        
        // This is a simplified read implementation. In a real app, 
        // you'd use a persistent reader loop to avoid lock issues.
        const reader = this.port.readable.getReader();
        
        try {
            while (Date.now() - startTime < timeoutMs) {
                const { value, done } = await reader.read();
                if (done) break;
                if (value) {
                    for (let b of value) buffer.push(b);
                    
                    // Look for header 0xFF 0xFF
                    if (buffer.length >= 6) {
                        const headerIdx = buffer.findIndex((b, i) => b === 0xFF && buffer[i+1] === 0xFF);
                        if (headerIdx !== -1) {
                            const pktStart = buffer.slice(headerIdx);
                            const pktLen = pktStart[3];
                            const totalExpected = pktLen + 4;
                            
                            if (pktStart.length >= totalExpected) {
                                reader.releaseLock();
                                return pktStart.slice(0, totalExpected);
                            }
                        }
                    }
                }
            }
        } finally {
            if (!reader.closed) reader.releaseLock();
        }
        return null;
    }

    // --- High Level API Methods ---

    /**
     * Ping a servo
     */
    async ping(id) {
        await this.sendPacket(id, STServoDef.INST_PING);
        const response = await this.receivePacket(id, 6);
        return !!response;
    }

    /**
     * Move servo to position with speed and acceleration
     */
    async writePosEx(id, position, speed, acc) {
        const params = [
            acc,
            this._loByte(position), this._hiByte(position),
            0, 0, // Time
            this._loByte(speed), this._hiByte(speed)
        ];
        // Address 41 is ACC, which starts the block
        return await this.write(id, STSAddress.ACC, params);
    }

    /**
     * Write data to memory address
     */
    async write(id, address, data) {
        const params = [address, ...data];
        return await this.sendPacket(id, STServoDef.INST_WRITE, params);
    }

    /**
     * Synchronous Write 1 byte
     */
    async write1Byte(id, address, value) {
        return await this.write(id, address, [value & 0xFF]);
    }

    /**
     * Enable or Disable Torque
     */
    async setTorque(id, enabled) {
        return await this.write1Byte(id, STSAddress.TORQUE_ENABLE, enabled ? 1 : 0);
    }

    /**
     * Read current position
     */
    async readPosition(id) {
        await this.sendPacket(id, STServoDef.INST_READ, [STSAddress.PRESENT_POSITION_L, 2]);
        const res = await this.receivePacket(id, 8);
        if (res && res.length >= 8) {
            // Header(2), ID(1), Len(1), Err(1), Param(2), Checksum(1)
            const pos = this._makeWord(res[5], res[6]);
            // Convert to signed 15-bit if needed (skipped for simplicity here)
            return pos;
        }
        return null;
    }
}
