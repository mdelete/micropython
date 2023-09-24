#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "spi_master.h"
#include "bsp.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "mcp_can.h"

#define TX_RX_MSG_LENGTH 10

static volatile bool m_transfer_completed = true;
uint8_t dummy_receive;
mcp_can_t m_mcp_can;

void spi_master_0_event_handler(spi_master_evt_t spi_master_evt)
{
    switch(spi_master_evt.evt_type)
    {
        case SPI_MASTER_EVT_TRANSFER_COMPLETED:
            //spi_master_close(SPI_MASTER_0);
            m_transfer_completed = true;
            break;
        default:
            break;
    }
}

static void spi_master_init(spi_master_hw_instance_t spi_master_instance, spi_master_event_handler_t spi_master_event_handler, const bool lsb)
{
    uint32_t err_code = NRF_SUCCESS;
    spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;
    spi_config.SPI_Pin_SCK 	= 4;
    spi_config.SPI_Pin_MISO = 2;
    spi_config.SPI_Pin_MOSI = 3;
    spi_config.SPI_Pin_SS	= 1;
    spi_config.SPI_CONFIG_ORDER = (lsb ? SPI_CONFIG_ORDER_LsbFirst : SPI_CONFIG_ORDER_MsbFirst);

    err_code = spi_master_open(spi_master_instance, &spi_config);
    APP_ERROR_CHECK(err_code);

    spi_master_evt_handler_reg(spi_master_instance, spi_master_event_handler);
}

static void spi_send_recv(const spi_master_hw_instance_t spi_master_hw_instance, uint8_t p_tx_data, uint8_t p_rx_data, const uint16_t len)
{
    uint32_t err;

  again:
    err = spi_master_send_recv(spi_master_hw_instance, &p_tx_data, len, &p_rx_data, len);
    
    if (err == NRF_ERROR_BUSY) {
        nrf_delay_ms(10);
        goto again;
    }

    APP_ERROR_CHECK(err);
}

void mcp_select()   
{
    nrf_gpio_pin_clear(m_mcp_can.m_cs);
}

void mcp_unselect()
{
    nrf_gpio_pin_set(m_mcp_can.m_cs);
}

void mcp_can_reset()
{
    mcp_select();
    spi_send_recv(SPI_MASTER_0, MCP_RESET, dummy_receive, 1);
    mcp_unselect();
    nrf_delay_ms(10);
}

uint8_t mcp2515_readRegister(const uint8_t address)
{
    uint8_t ret;
    mcp_select();
    spi_send_recv(SPI_MASTER_0, MCP_READ, dummy_receive, 1);
    spi_send_recv(SPI_MASTER_0, address, dummy_receive, 1);
    spi_send_recv(SPI_MASTER_0, 0x00, ret, 1);
    mcp_unselect();
    return ret;
}

void  mcp2515_readRegisterS(const uint8_t address, uint8_t values[], const uint8_t n)
{
	uint8_t i;
	mcp_select();
    spi_send_recv(SPI_MASTER_0, MCP_READ, dummy_receive, 1);
    spi_send_recv(SPI_MASTER_0, address, dummy_receive, 1);
	// mcp2515 has auto-increment of address-pointer
	for (i=0; i<n && i<CAN_MAX_CHAR_IN_MESSAGE; i++) {
		spi_send_recv(SPI_MASTER_0, 0x00, values[i], 1);
	}
	mcp_unselect();
}

void  mcp2515_setRegister(const uint8_t address, const uint8_t value)
{
    mcp_select();
    spi_send_recv(SPI_MASTER_0, MCP_WRITE, dummy_receive, 1);
    spi_send_recv(SPI_MASTER_0, address, dummy_receive, 1);
    spi_send_recv(SPI_MASTER_0, value, dummy_receive, 1);
    mcp_unselect();
}

void mcp2515_setRegisterS(const uint8_t address, const uint8_t values[], const uint8_t n)
{
    uint8_t i;
    mcp_select();
    spi_send_recv(SPI_MASTER_0, MCP_WRITE, dummy_receive, 1);
    spi_send_recv(SPI_MASTER_0, address, dummy_receive, 1);
       
    for (i=0; i<n; i++) 
    {
        spi_send_recv(SPI_MASTER_0, values[i], dummy_receive, 1);
    }
    mcp_unselect();
}

void  mcp2515_modifyRegister(const uint8_t address, const uint8_t mask, const uint8_t data)
{
    mcp_select();
    spi_send_recv(SPI_MASTER_0, MCP_BITMOD, dummy_receive, 1);
    spi_send_recv(SPI_MASTER_0, address, dummy_receive, 1);
    spi_send_recv(SPI_MASTER_0, mask, dummy_receive, 1);
    spi_send_recv(SPI_MASTER_0, data, dummy_receive, 1);
    mcp_unselect();
}

uint8_t  mcp2515_readStatus(void)
{
    uint8_t i;
    mcp_select();
    spi_send_recv(SPI_MASTER_0, MCP_READ_STATUS, dummy_receive, 1);
    spi_send_recv(SPI_MASTER_0, 0x00, i, 1);
    mcp_unselect();
    return i;
}

uint8_t mcp2515_setCANCTRL_Mode(const uint8_t newmode)
{
    uint8_t i;
    printf("mcp2515_setCANCTRL_Mode\n");
    mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);
    i = mcp2515_readRegister(MCP_CANCTRL);
    i &= MODE_MASK;
    if ( i == newmode ) 
    {
        return MCP2515_OK;
    }

    return MCP2515_FAIL;
}

uint8_t mcp2515_configRate(const uint8_t canSpeed)            
{
    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canSpeed) 
    {
        case (CAN_5KBPS):
        cfg1 = MCP_16MHz_5kBPS_CFG1;
        cfg2 = MCP_16MHz_5kBPS_CFG2;
        cfg3 = MCP_16MHz_5kBPS_CFG3;
        break;

        case (CAN_10KBPS):
        cfg1 = MCP_16MHz_10kBPS_CFG1;
        cfg2 = MCP_16MHz_10kBPS_CFG2;
        cfg3 = MCP_16MHz_10kBPS_CFG3;
        break;

        case (CAN_20KBPS):
        cfg1 = MCP_16MHz_20kBPS_CFG1;
        cfg2 = MCP_16MHz_20kBPS_CFG2;
        cfg3 = MCP_16MHz_20kBPS_CFG3;
        break;
        
        case (CAN_31K25BPS):
        cfg1 = MCP_16MHz_31k25BPS_CFG1;
        cfg2 = MCP_16MHz_31k25BPS_CFG2;
        cfg3 = MCP_16MHz_31k25BPS_CFG3;
        break;

        case (CAN_33KBPS):
        cfg1 = MCP_16MHz_33kBPS_CFG1;
        cfg2 = MCP_16MHz_33kBPS_CFG2;
        cfg3 = MCP_16MHz_33kBPS_CFG3;
        break;

        case (CAN_40KBPS):
        cfg1 = MCP_16MHz_40kBPS_CFG1;
        cfg2 = MCP_16MHz_40kBPS_CFG2;
        cfg3 = MCP_16MHz_40kBPS_CFG3;
        break;

        case (CAN_50KBPS):
        cfg1 = MCP_16MHz_50kBPS_CFG1;
        cfg2 = MCP_16MHz_50kBPS_CFG2;
        cfg3 = MCP_16MHz_50kBPS_CFG3;
        break;

        case (CAN_80KBPS):
        cfg1 = MCP_16MHz_80kBPS_CFG1;
        cfg2 = MCP_16MHz_80kBPS_CFG2;
        cfg3 = MCP_16MHz_80kBPS_CFG3;
        break;

        case (CAN_83K3BPS):
        cfg1 = MCP_16MHz_83k3BPS_CFG1;
        cfg2 = MCP_16MHz_83k3BPS_CFG2;
        cfg3 = MCP_16MHz_83k3BPS_CFG3;
        break;  

        case (CAN_95KBPS):
        cfg1 = MCP_16MHz_95kBPS_CFG1;
        cfg2 = MCP_16MHz_95kBPS_CFG2;
        cfg3 = MCP_16MHz_95kBPS_CFG3;
        break;

        case (CAN_100KBPS): /* 100KBPS */
        cfg1 = MCP_16MHz_100kBPS_CFG1;
        cfg2 = MCP_16MHz_100kBPS_CFG2;
        cfg3 = MCP_16MHz_100kBPS_CFG3;
        break;

        case (CAN_125KBPS):
        cfg1 = MCP_16MHz_125kBPS_CFG1;
        cfg2 = MCP_16MHz_125kBPS_CFG2;
        cfg3 = MCP_16MHz_125kBPS_CFG3;
        break;

        case (CAN_200KBPS):
        cfg1 = MCP_16MHz_200kBPS_CFG1;
        cfg2 = MCP_16MHz_200kBPS_CFG2;
        cfg3 = MCP_16MHz_200kBPS_CFG3;
        break;

        case (CAN_250KBPS):
        cfg1 = MCP_16MHz_250kBPS_CFG1;
        cfg2 = MCP_16MHz_250kBPS_CFG2;
        cfg3 = MCP_16MHz_250kBPS_CFG3;
        break;

        case (CAN_500KBPS):
        cfg1 = MCP_16MHz_500kBPS_CFG1;
        cfg2 = MCP_16MHz_500kBPS_CFG2;
        cfg3 = MCP_16MHz_500kBPS_CFG3;
        break;
        
        case (CAN_1000KBPS):
        cfg1 = MCP_16MHz_1000kBPS_CFG1;
        cfg2 = MCP_16MHz_1000kBPS_CFG2;
        cfg3 = MCP_16MHz_1000kBPS_CFG3;
        break;  

        default:
        set = 0;
        break;
    }

    if (set) {
        mcp2515_setRegister(MCP_CNF1, cfg1);
        mcp2515_setRegister(MCP_CNF2, cfg2);
        mcp2515_setRegister(MCP_CNF3, cfg3);
        return MCP2515_OK;
    }
    else {
        return MCP2515_FAIL;
    }
}

void mcp2515_initCANBuffers(void)
{
    uint8_t i, a1, a2, a3;

//    uint8_t std = 0;               
//    uint8_t ext = 1;
//    uint32_t ulMask = 0x00, ulFilt = 0x00;


    //mcp2515_write_id(MCP_RXM0SIDH, ext, ulMask);			/*Set both masks to 0           */
    //mcp2515_write_id(MCP_RXM1SIDH, ext, ulMask);			/*Mask register ignores ext bit */
                                                            /* Set all filters to 0         */
    //mcp2515_write_id(MCP_RXF0SIDH, ext, ulFilt);			/* RXB0: extended               */
    //mcp2515_write_id(MCP_RXF1SIDH, std, ulFilt);			/* RXB1: standard               */
    //mcp2515_write_id(MCP_RXF2SIDH, ext, ulFilt);			/* RXB2: extended               */
    //mcp2515_write_id(MCP_RXF3SIDH, std, ulFilt);			/* RXB3: standard               */
    //mcp2515_write_id(MCP_RXF4SIDH, ext, ulFilt);
    //mcp2515_write_id(MCP_RXF5SIDH, std, ulFilt);
                                                                        /* Clear, deactivate the three  */
                                                                        /* transmit buffers             */
                                                                        /* TXBnCTRL -> TXBnD7           */
    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++) {                                          /* in-buffer loop               */
        mcp2515_setRegister(a1, 0);
        mcp2515_setRegister(a2, 0);
        mcp2515_setRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
    mcp2515_setRegister(MCP_RXB0CTRL, 0);
    mcp2515_setRegister(MCP_RXB1CTRL, 0);
}

uint8_t mcp2515_init(const uint8_t canSpeed)
{
    uint8_t res;

    mcp_can_reset();

    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
    {
#if DEBUG_MODE
      printf("Enter setting mode fall\r\n");
#else
      nrf_delay_ms(10);
#endif
      return res;
    }
#if DEBUG_MODE
    printf("Enter setting mode success \r\n");
#else
    nrf_delay_ms(10);
#endif

    if(mcp2515_configRate(canSpeed))
    {
#if DEBUG_MODE
      printf("set rate fall!!\n");
#else
      nrf_delay_ms(10);
#endif
      return res;
    }
#if DEBUG_MODE
    printf("set rate success!!\n");
#else
    nrf_delay_ms(10);
#endif

    if ( res == MCP2515_OK ) {

                                                                        /* init canbuffers              */
        mcp2515_initCANBuffers();

                                                                        /* interrupt mode               */
        mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);

#if (DEBUG_RXANY==1)
                                                                        /* enable both receive-buffers  */
                                                                        /* to receive any message       */
                                                                        /* and enable rollover          */
        mcp2515_modifyRegister(MCP_RXB0CTRL,
        MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
        MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
        mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
        MCP_RXB_RX_ANY);
#else
                                                                        /* enable both receive-buffers  */
                                                                        /* to receive messages          */
                                                                        /* with std. and ext. identifie */
                                                                        /* rs                           */
                                                                        /* and enable rollover          */
        mcp2515_modifyRegister(MCP_RXB0CTRL,
        MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
        MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK );
        mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
        MCP_RXB_RX_STDEXT);
#endif
                                                                        /* enter normal mode            */
        res = mcp2515_setCANCTRL_Mode(MODE_NORMAL);                                                                
        if(res)
        {
#if DEBUG_MODE
          printf("Enter Normal Mode Fall!!\n");
#else
            nrf_delay_ms(10);
#endif
            return res;
        }


#if DEBUG_MODE
          printf("Enter Normal Mode Success!!\n");
#else
            nrf_delay_ms(10);
#endif

    }
    return res;

}

void  mcp2515_write_id( const uint8_t mcp_addr, const uint8_t ext, const uint32_t id )
{
    uint16_t canid;
    uint8_t tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) 
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 3 );
        tbufdata[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    mcp2515_setRegisterS( mcp_addr, tbufdata, 4 );
}

void  mcp2515_read_id( const uint8_t mcp_addr, uint8_t* ext, uint32_t* id )
{
    uint8_t tbufdata[4];

    *ext = 0;
    *id = 0;

    mcp2515_readRegisterS( mcp_addr, tbufdata, 4 );

    *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M ) 
    {
                                                                        /* extended id                  */
        *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id<<8) + tbufdata[MCP_EID8];
        *id = (*id<<8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

void mcp2515_write_canMsg( const uint8_t buffer_sidh_addr)
{
    uint8_t mcp_addr;
    mcp_addr = buffer_sidh_addr;
    mcp2515_setRegisterS(mcp_addr+5, m_mcp_can.m_data, m_mcp_can.m_len );       /* write data bytes             */
    mcp2515_setRegister((mcp_addr+4), m_mcp_can.m_len );                        /* write the RTR and DLC        */
    mcp2515_write_id(mcp_addr, m_mcp_can.m_ext_flag, m_mcp_can.m_id );          /* write CAN id                 */
}

void mcp2515_read_canMsg(const uint8_t buffer_sidh_addr)        /* read can msg                 */
{
    uint8_t mcp_addr, ctrl;

    mcp_addr = buffer_sidh_addr;

    mcp2515_read_id( mcp_addr, &m_mcp_can.m_ext_flag, &m_mcp_can.m_id );

    ctrl = mcp2515_readRegister( mcp_addr-1 );
    m_mcp_can.m_len = mcp2515_readRegister( mcp_addr+4 );
    m_mcp_can.m_len &= MCP_DLC_MASK;
    mcp2515_readRegisterS( mcp_addr+5, &(m_mcp_can.m_data[0]), m_mcp_can.m_len );
}

void mcp2515_start_transmit(const uint8_t mcp_addr)              /* start transmit               */
{
    mcp2515_modifyRegister( mcp_addr-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );
}

uint8_t mcp2515_getNextFreeTXBuf(uint8_t *txbuf_n) /* get Next free txbuf */
{
    uint8_t res = MCP_ALLTXBUSY, ctrlval;
    uint8_t ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

    *txbuf_n = 0x00;

    for (int i = 0; i < MCP_N_TXBUFFERS; i++)
    {
        ctrlval = mcp2515_readRegister( ctrlregs[i] );
        if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 )
        {
            *txbuf_n = ctrlregs[i] + 1;
            res = MCP2515_OK;
            break;
        }
    }

    return res;
}

uint8_t setMsg(uint32_t id, uint8_t ext, uint8_t len, uint8_t *pData)
{
    m_mcp_can.m_id       = id;
    m_mcp_can.m_len      = len;
    m_mcp_can.m_ext_flag = ext;
    for(int i = 0; i < MAX_CHAR_IN_MESSAGE; i++) // FIXME: len statt MAX_CHAR_IN_MESSAGE?
    {
        m_mcp_can.m_data[i] = *(pData+i);
    }
    return MCP2515_OK;
}

uint8_t clearMsg()
{
    m_mcp_can.m_id       = 0;
    m_mcp_can.m_len      = 0;
    m_mcp_can.m_ext_flag = 0;
    for(int i = 0; i < m_mcp_can.m_len; i++)
    {
        m_mcp_can.m_data[i] = 0x00;
    }
    return MCP2515_OK;
}

uint8_t sendMsg()
{
    uint8_t res, res1, txbuf_n;
    uint16_t uiTimeOut = 0;

    do {
        res = mcp2515_getNextFreeTXBuf(&txbuf_n); /* info = addr. */
        uiTimeOut++;
    } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

    if(uiTimeOut == TIMEOUTVALUE) 
    {
        return CAN_GETTXBFTIMEOUT; /* get tx buff time out */
    }
    uiTimeOut = 0;
    mcp2515_write_canMsg( txbuf_n);
    mcp2515_start_transmit( txbuf_n );
    do {
        uiTimeOut++;        
        res1= mcp2515_readRegister(txbuf_n-1 /* the ctrl reg is located at txbuf_n-1 */);  /* read send buff ctrl reg 	*/
        res1 = res1 & 0x08;
    } while(res1 && (uiTimeOut < TIMEOUTVALUE));   
    if(uiTimeOut == TIMEOUTVALUE) /* send msg timeout */	
    {
        return CAN_SENDMSGTIMEOUT;
    }
    return CAN_OK;
}

uint8_t readMsg()
{
    uint8_t stat, res;

    stat = mcp2515_readStatus();

    if ( stat & MCP_STAT_RX0IF ) /* Msg in Buffer 0              */
    {
        mcp2515_read_canMsg(MCP_RXBUF_0);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if ( stat & MCP_STAT_RX1IF ) /* Msg in Buffer 1              */
    {
        mcp2515_read_canMsg(MCP_RXBUF_1);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else 
    {
        res = CAN_NOMSG;
    }
    return res;
}

void mcp_can_setcs(uint8_t cs)
{
    m_mcp_can.m_cs = cs;
    nrf_gpio_cfg_output(m_mcp_can.m_cs);
    mcp_unselect();
}

uint8_t mcp_can_begin(uint8_t speedset)
{
    spi_master_init(SPI_MASTER_0, spi_master_0_event_handler, 1);
    uint8_t res = mcp2515_init(speedset);
    if (res == MCP2515_OK)
        return CAN_OK;
    else
        return CAN_FAILINIT;
}

uint8_t mcp_can_send_msg(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    setMsg(id, ext, len, buf);
    return sendMsg();
}

uint8_t mcp_can_read_msg(uint32_t *ID, uint8_t *len, uint8_t buf[])
{
    uint8_t rc;
    rc = readMsg();

    if (rc == CAN_OK)
    {
       *len = m_mcp_can.m_len;
       *ID  =  m_mcp_can.m_id;
       for(int i = 0; i<m_mcp_can.m_len && i < MAX_CHAR_IN_MESSAGE; i++)
       {
           buf[i] = m_mcp_can.m_data[i];
       }
    }
    else
    {
       *len = 0;
    }
    return rc;
}

uint8_t mcp_can_check_receive(void)
{
    uint8_t res = mcp2515_readStatus(); /* RXnIF in Bit 1 and 0 */

    if ( res & MCP_STAT_RXIF_MASK ) 
    {
        return CAN_MSGAVAIL;
    }
    else 
    {
        return CAN_NOMSG;
    }
}

uint8_t mcp_can_check_error(void)
{
    uint8_t eflg = mcp2515_readRegister(MCP_EFLG);

    if ( eflg & MCP_EFLG_ERRORMASK ) 
    {
        return CAN_CTRLERROR;
    }
    else 
    {
        return CAN_OK;
    }
}
