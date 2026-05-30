#include "mcp25625.hpp"

#include <cstring>

#include <pico/stdlib.h>

const struct MCP25625::TXBn_REGS MCP25625::TXB[MCP25625::N_TXBUFFERS] = {
    {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA},
    {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
    {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}
};

const struct MCP25625::RXBn_REGS MCP25625::RXB[N_RXBUFFERS] = {
    {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF},
    {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF}
};

MCP25625::MCP25625(
    spi_inst_t * CHANNEL,
    uint8_t      CS_PIN,
    uint8_t      TX_PIN,
    uint8_t      RX_PIN,
    uint8_t      SCK_PIN,
    uint32_t     SPI_CLOCK
) {
	this->SPI_CHANNEL = CHANNEL;
	spi_init(this->SPI_CHANNEL, SPI_CLOCK);
	gpio_set_function(TX_PIN, GPIO_FUNC_SPI);
	gpio_set_function(RX_PIN, GPIO_FUNC_SPI);
	gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);
	spi_set_format(this->SPI_CHANNEL, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

	this->SPI_CS_PIN = CS_PIN;
	gpio_init(this->SPI_CS_PIN);
	gpio_set_dir(this->SPI_CS_PIN, GPIO_OUT);

	endSPI();
}

inline void MCP25625::startSPI() {
	asm volatile("nop \n nop \n nop");
	gpio_put(this->SPI_CS_PIN, 0);
	asm volatile("nop \n nop \n nop");
}

inline void MCP25625::endSPI() {
	asm volatile("nop \n nop \n nop");
	gpio_put(this->SPI_CS_PIN, 1);
	asm volatile("nop \n nop \n nop");
}

MCP25625::ERROR MCP25625::reset(void) {
	startSPI();

	uint8_t instruction = INSTRUCTION_RESET;
	spi_write_blocking(this->SPI_CHANNEL, &instruction, 1);

	endSPI();

	// Depends on oscillator & capacitors used
	sleep_ms(10);

	uint8_t zeros[14];
	memset(zeros, 0, sizeof(zeros));
	setRegisters(MCP_TXB0CTRL, zeros, 14);
	setRegisters(MCP_TXB1CTRL, zeros, 14);
	setRegisters(MCP_TXB2CTRL, zeros, 14);

	setRegister(MCP_RXB0CTRL, 0);
	setRegister(MCP_RXB1CTRL, 0);

	setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

	// receives all valid messages using either Standard or Extended Identifiers that
	// meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
	modifyRegister(
	    MCP_RXB0CTRL, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
	    RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT
	);
	modifyRegister(
	    MCP_RXB1CTRL, RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
	    RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT
	);

	// clear filters and masks
	// do not filter any standard frames for RXF0 used by RXB0
	// do not filter any extended frames for RXF1 used by RXB1
	RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
	for (int i = 0; i < 6; i++) {
		bool  ext    = (i == 1);
		ERROR result = setFilter(filters[i], ext, 0);
		if (result != ERROR_OK) { return result; }
	}

	MASK masks[] = {MASK0, MASK1};
	for (int i = 0; i < 2; i++) {
		ERROR result = setFilterMask(masks[i], true, 0);
		if (result != ERROR_OK) { return result; }
	}

	return ERROR_OK;
}

uint8_t MCP25625::readRegister(const REGISTER reg) {
	startSPI();

	uint8_t data[2] = {INSTRUCTION_READ, reg};

	spi_write_blocking(this->SPI_CHANNEL, data, 2);

	uint8_t ret;
	spi_read_blocking(this->SPI_CHANNEL, 0x00, &ret, 1);

	endSPI();

	return ret;
}

void MCP25625::readRegisters(const REGISTER reg, uint8_t values[], uint8_t const n) {
	startSPI();

	uint8_t data[2] = {INSTRUCTION_READ, reg};
	spi_write_blocking(this->SPI_CHANNEL, data, 2);

	spi_read_blocking(this->SPI_CHANNEL, 0x00, values, n);

	endSPI();
}

void MCP25625::setRegister(const REGISTER reg, uint8_t const value) {
	startSPI();

	uint8_t data[3] = {INSTRUCTION_WRITE, reg, value};
	spi_write_blocking(this->SPI_CHANNEL, data, 3);

	endSPI();
}

void MCP25625::setRegisters(const REGISTER reg, uint8_t const values[], uint8_t const n) {
	startSPI();

	uint8_t data[2] = {INSTRUCTION_WRITE, reg};
	spi_write_blocking(this->SPI_CHANNEL, data, 2);

	spi_write_blocking(this->SPI_CHANNEL, values, n);

	endSPI();
}

void MCP25625::modifyRegister(const REGISTER reg, uint8_t const mask, uint8_t const data) {
	startSPI();

	uint8_t d[4] = {INSTRUCTION_BITMOD, reg, mask, data};

	spi_write_blocking(this->SPI_CHANNEL, d, 4);

	endSPI();
}

uint8_t MCP25625::getStatus(void) {
	startSPI();

	uint8_t instruction = INSTRUCTION_READ_STATUS;
	spi_write_blocking(this->SPI_CHANNEL, &instruction, 1);

	uint8_t ret;
	spi_read_blocking(this->SPI_CHANNEL, 0x00, &ret, 1);

	endSPI();

	return ret;
}

MCP25625::ERROR MCP25625::setConfigMode() {
	return setMode(CANCTRL_REQOP_CONFIG);
}

MCP25625::ERROR MCP25625::setListenOnlyMode() {
	return setMode(CANCTRL_REQOP_LISTENONLY);
}

MCP25625::ERROR MCP25625::setSleepMode() {
	return setMode(CANCTRL_REQOP_SLEEP);
}

MCP25625::ERROR MCP25625::setLoopbackMode() {
	return setMode(CANCTRL_REQOP_LOOPBACK);
}

MCP25625::ERROR MCP25625::setNormalMode() {
	return setMode(CANCTRL_REQOP_NORMAL);
}

MCP25625::ERROR MCP25625::setMode(const CANCTRL_REQOP_MODE mode) {
	modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode);

	unsigned long endTime   = to_ms_since_boot(get_absolute_time()) + 10;
	bool          modeMatch = false;
	while (to_ms_since_boot(get_absolute_time()) < endTime) {
		uint8_t newmode = readRegister(MCP_CANSTAT);
		newmode &= CANSTAT_OPMOD;

		modeMatch = newmode == mode;

		if (modeMatch) { break; }
	}

	return modeMatch ? ERROR_OK : ERROR_FAIL;
}

MCP25625::ERROR MCP25625::setBitrate(const CAN_SPEED canSpeed) {
	return setBitrate(canSpeed, MCP_16MHZ);
}

MCP25625::ERROR MCP25625::setBitrate(const CAN_SPEED canSpeed, CAN_CLOCK canClock) {
	ERROR error = setConfigMode();
	if (error != ERROR_OK) { return error; }

	uint8_t set, cfg1, cfg2, cfg3;
	set = 1;
	switch (canClock) {
	case (MCP_8MHZ):
		switch (canSpeed) {
		case (CAN_5KBPS):  // 5KBPS
			cfg1 = MCP_8MHz_5kBPS_CFG1;
			cfg2 = MCP_8MHz_5kBPS_CFG2;
			cfg3 = MCP_8MHz_5kBPS_CFG3;
			break;

		case (CAN_10KBPS):  // 10KBPS
			cfg1 = MCP_8MHz_10kBPS_CFG1;
			cfg2 = MCP_8MHz_10kBPS_CFG2;
			cfg3 = MCP_8MHz_10kBPS_CFG3;
			break;

		case (CAN_20KBPS):  // 20KBPS
			cfg1 = MCP_8MHz_20kBPS_CFG1;
			cfg2 = MCP_8MHz_20kBPS_CFG2;
			cfg3 = MCP_8MHz_20kBPS_CFG3;
			break;

		case (CAN_31K25BPS):  // 31.25KBPS
			cfg1 = MCP_8MHz_31k25BPS_CFG1;
			cfg2 = MCP_8MHz_31k25BPS_CFG2;
			cfg3 = MCP_8MHz_31k25BPS_CFG3;
			break;

		case (CAN_33KBPS):  // 33.333KBPS
			cfg1 = MCP_8MHz_33k3BPS_CFG1;
			cfg2 = MCP_8MHz_33k3BPS_CFG2;
			cfg3 = MCP_8MHz_33k3BPS_CFG3;
			break;

		case (CAN_40KBPS):  // 40Kbps
			cfg1 = MCP_8MHz_40kBPS_CFG1;
			cfg2 = MCP_8MHz_40kBPS_CFG2;
			cfg3 = MCP_8MHz_40kBPS_CFG3;
			break;

		case (CAN_50KBPS):  // 50Kbps
			cfg1 = MCP_8MHz_50kBPS_CFG1;
			cfg2 = MCP_8MHz_50kBPS_CFG2;
			cfg3 = MCP_8MHz_50kBPS_CFG3;
			break;

		case (CAN_80KBPS):  // 80Kbps
			cfg1 = MCP_8MHz_80kBPS_CFG1;
			cfg2 = MCP_8MHz_80kBPS_CFG2;
			cfg3 = MCP_8MHz_80kBPS_CFG3;
			break;

		case (CAN_100KBPS):  // 100Kbps
			cfg1 = MCP_8MHz_100kBPS_CFG1;
			cfg2 = MCP_8MHz_100kBPS_CFG2;
			cfg3 = MCP_8MHz_100kBPS_CFG3;
			break;

		case (CAN_125KBPS):  // 125Kbps
			cfg1 = MCP_8MHz_125kBPS_CFG1;
			cfg2 = MCP_8MHz_125kBPS_CFG2;
			cfg3 = MCP_8MHz_125kBPS_CFG3;
			break;

		case (CAN_200KBPS):  // 200Kbps
			cfg1 = MCP_8MHz_200kBPS_CFG1;
			cfg2 = MCP_8MHz_200kBPS_CFG2;
			cfg3 = MCP_8MHz_200kBPS_CFG3;
			break;

		case (CAN_250KBPS):  // 250Kbps
			cfg1 = MCP_8MHz_250kBPS_CFG1;
			cfg2 = MCP_8MHz_250kBPS_CFG2;
			cfg3 = MCP_8MHz_250kBPS_CFG3;
			break;

		case (CAN_500KBPS):  // 500Kbps
			cfg1 = MCP_8MHz_500kBPS_CFG1;
			cfg2 = MCP_8MHz_500kBPS_CFG2;
			cfg3 = MCP_8MHz_500kBPS_CFG3;
			break;

		case (CAN_1000KBPS):  // 1Mbps
			cfg1 = MCP_8MHz_1000kBPS_CFG1;
			cfg2 = MCP_8MHz_1000kBPS_CFG2;
			cfg3 = MCP_8MHz_1000kBPS_CFG3;
			break;

		default:
			set = 0;
			break;
		}
		break;

	case (MCP_16MHZ):
		switch (canSpeed) {
		case (CAN_5KBPS):  // 5Kbps
			cfg1 = MCP_16MHz_5kBPS_CFG1;
			cfg2 = MCP_16MHz_5kBPS_CFG2;
			cfg3 = MCP_16MHz_5kBPS_CFG3;
			break;

		case (CAN_10KBPS):  // 10Kbps
			cfg1 = MCP_16MHz_10kBPS_CFG1;
			cfg2 = MCP_16MHz_10kBPS_CFG2;
			cfg3 = MCP_16MHz_10kBPS_CFG3;
			break;

		case (CAN_20KBPS):  // 20Kbps
			cfg1 = MCP_16MHz_20kBPS_CFG1;
			cfg2 = MCP_16MHz_20kBPS_CFG2;
			cfg3 = MCP_16MHz_20kBPS_CFG3;
			break;

		case (CAN_33KBPS):  // 33.333Kbps
			cfg1 = MCP_16MHz_33k3BPS_CFG1;
			cfg2 = MCP_16MHz_33k3BPS_CFG2;
			cfg3 = MCP_16MHz_33k3BPS_CFG3;
			break;

		case (CAN_40KBPS):  // 40Kbps
			cfg1 = MCP_16MHz_40kBPS_CFG1;
			cfg2 = MCP_16MHz_40kBPS_CFG2;
			cfg3 = MCP_16MHz_40kBPS_CFG3;
			break;

		case (CAN_50KBPS):  // 50Kbps
			cfg1 = MCP_16MHz_50kBPS_CFG1;
			cfg2 = MCP_16MHz_50kBPS_CFG2;
			cfg3 = MCP_16MHz_50kBPS_CFG3;
			break;

		case (CAN_80KBPS):  // 80Kbps
			cfg1 = MCP_16MHz_80kBPS_CFG1;
			cfg2 = MCP_16MHz_80kBPS_CFG2;
			cfg3 = MCP_16MHz_80kBPS_CFG3;
			break;

		case (CAN_83K3BPS):  // 83.333Kbps
			cfg1 = MCP_16MHz_83k3BPS_CFG1;
			cfg2 = MCP_16MHz_83k3BPS_CFG2;
			cfg3 = MCP_16MHz_83k3BPS_CFG3;
			break;

		case (CAN_100KBPS):  // 100Kbps
			cfg1 = MCP_16MHz_100kBPS_CFG1;
			cfg2 = MCP_16MHz_100kBPS_CFG2;
			cfg3 = MCP_16MHz_100kBPS_CFG3;
			break;

		case (CAN_125KBPS):  // 125Kbps
			cfg1 = MCP_16MHz_125kBPS_CFG1;
			cfg2 = MCP_16MHz_125kBPS_CFG2;
			cfg3 = MCP_16MHz_125kBPS_CFG3;
			break;

		case (CAN_200KBPS):  // 200Kbps
			cfg1 = MCP_16MHz_200kBPS_CFG1;
			cfg2 = MCP_16MHz_200kBPS_CFG2;
			cfg3 = MCP_16MHz_200kBPS_CFG3;
			break;

		case (CAN_250KBPS):  // 250Kbps
			cfg1 = MCP_16MHz_250kBPS_CFG1;
			cfg2 = MCP_16MHz_250kBPS_CFG2;
			cfg3 = MCP_16MHz_250kBPS_CFG3;
			break;

		case (CAN_500KBPS):  // 500Kbps
			cfg1 = MCP_16MHz_500kBPS_CFG1;
			cfg2 = MCP_16MHz_500kBPS_CFG2;
			cfg3 = MCP_16MHz_500kBPS_CFG3;
			break;

		case (CAN_1000KBPS):  // 1Mbps
			cfg1 = MCP_16MHz_1000kBPS_CFG1;
			cfg2 = MCP_16MHz_1000kBPS_CFG2;
			cfg3 = MCP_16MHz_1000kBPS_CFG3;
			break;

		default:
			set = 0;
			break;
		}
		break;

	case (MCP_20MHZ):
		switch (canSpeed) {
		case (CAN_33KBPS):  // 33.333Kbps
			cfg1 = MCP_20MHz_33k3BPS_CFG1;
			cfg2 = MCP_20MHz_33k3BPS_CFG2;
			cfg3 = MCP_20MHz_33k3BPS_CFG3;
			break;

		case (CAN_40KBPS):  // 40Kbps
			cfg1 = MCP_20MHz_40kBPS_CFG1;
			cfg2 = MCP_20MHz_40kBPS_CFG2;
			cfg3 = MCP_20MHz_40kBPS_CFG3;
			break;

		case (CAN_50KBPS):  // 50Kbps
			cfg1 = MCP_20MHz_50kBPS_CFG1;
			cfg2 = MCP_20MHz_50kBPS_CFG2;
			cfg3 = MCP_20MHz_50kBPS_CFG3;
			break;

		case (CAN_80KBPS):  // 80Kbps
			cfg1 = MCP_20MHz_80kBPS_CFG1;
			cfg2 = MCP_20MHz_80kBPS_CFG2;
			cfg3 = MCP_20MHz_80kBPS_CFG3;
			break;

		case (CAN_83K3BPS):  // 83.333Kbps
			cfg1 = MCP_20MHz_83k3BPS_CFG1;
			cfg2 = MCP_20MHz_83k3BPS_CFG2;
			cfg3 = MCP_20MHz_83k3BPS_CFG3;
			break;

		case (CAN_100KBPS):  // 100Kbps
			cfg1 = MCP_20MHz_100kBPS_CFG1;
			cfg2 = MCP_20MHz_100kBPS_CFG2;
			cfg3 = MCP_20MHz_100kBPS_CFG3;
			break;

		case (CAN_125KBPS):  // 125Kbps
			cfg1 = MCP_20MHz_125kBPS_CFG1;
			cfg2 = MCP_20MHz_125kBPS_CFG2;
			cfg3 = MCP_20MHz_125kBPS_CFG3;
			break;

		case (CAN_200KBPS):  // 200Kbps
			cfg1 = MCP_20MHz_200kBPS_CFG1;
			cfg2 = MCP_20MHz_200kBPS_CFG2;
			cfg3 = MCP_20MHz_200kBPS_CFG3;
			break;

		case (CAN_250KBPS):  // 250Kbps
			cfg1 = MCP_20MHz_250kBPS_CFG1;
			cfg2 = MCP_20MHz_250kBPS_CFG2;
			cfg3 = MCP_20MHz_250kBPS_CFG3;
			break;

		case (CAN_500KBPS):  // 500Kbps
			cfg1 = MCP_20MHz_500kBPS_CFG1;
			cfg2 = MCP_20MHz_500kBPS_CFG2;
			cfg3 = MCP_20MHz_500kBPS_CFG3;
			break;

		case (CAN_1000KBPS):  // 1Mbps
			cfg1 = MCP_20MHz_1000kBPS_CFG1;
			cfg2 = MCP_20MHz_1000kBPS_CFG2;
			cfg3 = MCP_20MHz_1000kBPS_CFG3;
			break;

		default:
			set = 0;
			break;
		}
		break;

	default:
		set = 0;
		break;
	}

	if (set) {
		setRegister(MCP_CNF1, cfg1);
		setRegister(MCP_CNF2, cfg2);
		setRegister(MCP_CNF3, cfg3);
		return ERROR_OK;
	} else {
		return ERROR_FAIL;
	}
}

MCP25625::ERROR MCP25625::setClkOut(const CAN_CLKOUT divisor) {
	if (divisor == CLKOUT_DISABLE) {
		/* Turn off CLKEN */
		modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, 0x00);

		/* Turn on CLKOUT for SOF */
		modifyRegister(MCP_CNF3, CNF3_SOF, CNF3_SOF);
		return ERROR_OK;
	}

	/* Set the prescaler (CLKPRE) */
	modifyRegister(MCP_CANCTRL, CANCTRL_CLKPRE, divisor);

	/* Turn on CLKEN */
	modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN);

	/* Turn off CLKOUT for SOF */
	modifyRegister(MCP_CNF3, CNF3_SOF, 0x00);
	return ERROR_OK;
}

void MCP25625::prepareId(uint8_t * buffer, bool const ext, uint32_t const id) {
	int32_t canid = (int32_t)(id & 0x0FFFF);

	if (ext) {
		buffer[MCP_EID0] = (uint8_t)(canid & 0xFF);
		buffer[MCP_EID8] = (uint8_t)(canid >> 8);
		canid            = (int32_t)(id >> 16);
		buffer[MCP_SIDL] = (uint8_t)(canid & 0x03);
		buffer[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3);
		buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
		buffer[MCP_SIDH] = (uint8_t)(canid >> 5);
	} else {
		buffer[MCP_SIDH] = (uint8_t)(canid >> 3);
		buffer[MCP_SIDL] = (uint8_t)((canid & 0x07) << 5);
		buffer[MCP_EID0] = 0;
		buffer[MCP_EID8] = 0;
	}
}

MCP25625::ERROR MCP25625::setFilterMask(const MASK mask, bool const ext, uint32_t const ulData) {
	ERROR res = setConfigMode();
	if (res != ERROR_OK) { return res; }

	uint8_t tbufdata[4];
	prepareId(tbufdata, ext, ulData);

	REGISTER reg;
	switch (mask) {
	case MASK0:
		reg = MCP_RXM0SIDH;
		break;
	case MASK1:
		reg = MCP_RXM1SIDH;
		break;
	default:
		return ERROR_FAIL;
	}

	setRegisters(reg, tbufdata, 4);

	return ERROR_OK;
}

MCP25625::ERROR MCP25625::setFilter(const RXF num, bool const ext, uint32_t const ulData) {
	ERROR res = setConfigMode();
	if (res != ERROR_OK) { return res; }

	REGISTER reg;

	switch (num) {
	case RXF0:
		reg = MCP_RXF0SIDH;
		break;
	case RXF1:
		reg = MCP_RXF1SIDH;
		break;
	case RXF2:
		reg = MCP_RXF2SIDH;
		break;
	case RXF3:
		reg = MCP_RXF3SIDH;
		break;
	case RXF4:
		reg = MCP_RXF4SIDH;
		break;
	case RXF5:
		reg = MCP_RXF5SIDH;
		break;
	default:
		return ERROR_FAIL;
	}

	uint8_t tbufdata[4];
	prepareId(tbufdata, ext, ulData);
	setRegisters(reg, tbufdata, 4);

	return ERROR_OK;
}

MCP25625::ERROR MCP25625::sendMessage(TXBn const txbn, const struct can_frame * frame) {
	if (frame->can_dlc > CAN_MAX_DLEN) { return ERROR_FAILTX; }

	const struct TXBn_REGS * txbuf = &TXB[txbn];

	uint8_t data[13];

	bool     ext = (frame->can_id & CAN_EFF_FLAG);
	bool     rtr = (frame->can_id & CAN_RTR_FLAG);
	uint32_t id  = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

	prepareId(data, ext, id);

	data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

	memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

	setRegisters(txbuf->SIDH, data, 5 + frame->can_dlc);

	modifyRegister(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

	uint8_t ctrl = readRegister(txbuf->CTRL);
	if ((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0) { return ERROR_FAILTX; }
	return ERROR_OK;
}

MCP25625::ERROR MCP25625::sendMessage(const struct can_frame * frame) {
	if (frame->can_dlc > CAN_MAX_DLEN) { return ERROR_FAILTX; }

	TXBn txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};

	for (int i = 0; i < N_TXBUFFERS; i++) {
		const struct TXBn_REGS * txbuf   = &TXB[txBuffers[i]];
		uint8_t                  ctrlval = readRegister(txbuf->CTRL);
		if ((ctrlval & TXB_TXREQ) == 0) { return sendMessage(txBuffers[i], frame); }
	}

	return ERROR_ALLTXBUSY;
}

MCP25625::ERROR MCP25625::readMessage(RXBn const rxbn, struct can_frame * frame) {
	const struct RXBn_REGS * rxb = &RXB[rxbn];

	uint8_t tbufdata[5];

	readRegisters(rxb->SIDH, tbufdata, 5);

	uint32_t id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);

	if ((tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) == TXB_EXIDE_MASK) {
		id = (id << 2) + (tbufdata[MCP_SIDL] & 0x03);
		id = (id << 8) + tbufdata[MCP_EID8];
		id = (id << 8) + tbufdata[MCP_EID0];
		id |= CAN_EFF_FLAG;
	}

	uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
	if (dlc > CAN_MAX_DLEN) { return ERROR_FAIL; }

	uint8_t ctrl = readRegister(rxb->CTRL);
	if (ctrl & RXBnCTRL_RTR) { id |= CAN_RTR_FLAG; }

	frame->can_id  = id;
	frame->can_dlc = dlc;

	readRegisters(rxb->DATA, frame->data, dlc);

	modifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

	return ERROR_OK;
}

MCP25625::ERROR MCP25625::readMessage(struct can_frame * frame) {
	ERROR   rc;
	uint8_t stat = getStatus();

	if (stat & STAT_RX0IF) {
		rc = readMessage(RXB0, frame);
	} else if (stat & STAT_RX1IF) {
		rc = readMessage(RXB1, frame);
	} else {
		rc = ERROR_NOMSG;
	}

	return rc;
}

bool MCP25625::checkReceive(void) {
	uint8_t res = getStatus();
	if (res & STAT_RXIF_MASK) {
		return true;
	} else {
		return false;
	}
}

bool MCP25625::checkError(void) {
	uint8_t eflg = getErrorFlags();

	if (eflg & EFLG_ERRORMASK) {
		return true;
	} else {
		return false;
	}
}

uint8_t MCP25625::getErrorFlags(void) {
	return readRegister(MCP_EFLG);
}

void MCP25625::clearRXnOVRFlags(void) {
	modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
}

uint8_t MCP25625::getInterrupts(void) {
	return readRegister(MCP_CANINTF);
}

void MCP25625::clearInterrupts(void) {
	setRegister(MCP_CANINTF, 0);
}

uint8_t MCP25625::getInterruptMask(void) {
	return readRegister(MCP_CANINTE);
}

void MCP25625::clearTXInterrupts(void) {
	modifyRegister(MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
}

void MCP25625::clearRXnOVR(void) {
	uint8_t eflg = getErrorFlags();
	if (eflg != 0) {
		clearRXnOVRFlags();
		clearInterrupts();
		// modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
	}
}

void MCP25625::clearMERR() {
	// modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
	// clearInterrupts();
	modifyRegister(MCP_CANINTF, CANINTF_MERRF, 0);
}

void MCP25625::clearERRIF() {
	// modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
	// clearInterrupts();
	modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
}

uint8_t MCP25625::errorCountRX(void) {
	return readRegister(MCP_REC);
}

uint8_t MCP25625::errorCountTX(void) {
	return readRegister(MCP_TEC);
}
