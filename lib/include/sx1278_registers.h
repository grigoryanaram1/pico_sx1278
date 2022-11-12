#ifndef __SX1278__REGISTERS__H__
#define __SX1278__REGISTERS__H__

#define REG_FIFO                                    0x00
#define REG_OP_MODE                                 0x01
#define REG_FR_MSB                                  0x06
#define REG_FR_MID                                  0x07
#define REG_FR_LSB                                  0x08
#define REG_PA_CONFIG                               0x09
#define REG_PA_RAMP                                 0x0A
#define REG_OCP                                     0x0B
#define REG_LNA                                     0x0C
#define REG_FIFO_ADDR_PTR                           0x0D
#define REG_FIFO_TX_BASE_ADDR                       0x0E
#define REG_FIFO_RX_BASE_ADDR                       0x0F
#define REG_FIFO_RX_CURRENT_ADDR                    0x10
#define REG_IRQ_FLAGS_MASK                          0x11
#define REG_IRQ_FLAGS                               0x12
#define REG_RX_NB_BYTES                             0x13
#define LR_RegRxHeaderCntValueMsb                   0x14
#define LR_RegRxHeaderCntValueLsb                   0x15
#define LR_RegRxPacketCntValueMsb                   0x16
#define LR_RegRxPacketCntValueLsb                   0x17
#define REG_MODEM_STATUS                            0x18
#define REG_PKT_SNR_VALUE                           0x19
#define REG_PKT_RSSI_VALUE                          0x1A
#define REG_RSSI_VALUE                              0x1B
#define REG_HOP_CHANNEL                             0x1C
#define REG_MODEM_CONFIG_1                          0x1D
#define REG_MODEM_CONFIG_2                          0x1E
#define REG_SYMB_TIMEOUT_LSB                        0x1F
#define REG_PREAMBLE_MSB                            0x20
#define REG_PREAMBLE_LSB                            0x21
#define REG_PAYLOAD_LENGTH                          0x22
#define REG_MAX_PAYLOAD_LENGTH                      0x23
#define REG_HOP_PERIOD                              0x24
#define REG_FIFO_RX_BYTE_ADDR                       0x25
#define REG_MODEM_CONFIG_3                          0x26
#define REG_DIOMAPPING1                             0x40
#define REG_DIOMAPPING2                             0x41
#define REG_VERSION                                 0x42
#define REG_PLLHOP                                  0x44
#define REG_TCXO                                    0x4B
#define REG_PADAC                                   0x4D
#define REG_FORMERTEMP                              0x5B
#define REG_AGCREF                                  0x61
#define REG_AGCTHRESH1                              0x62
#define REG_AGCTHRESH2                              0x63
#define REG_AGCTHRESH3                              0x64

#endif /* __SX1278__REGISTERS__H__ */