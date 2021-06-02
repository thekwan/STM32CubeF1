#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

typedef struct _spi_tool_t
{
      int fd;
      uint8_t mode;
      uint8_t bits;
      uint32_t hz;
      struct spi_ioc_transfer xfer[2];

}spi_tool_t;

void spi_deinit(spi_tool_t *spi_tool)
{
      if (spi_tool)
      {
            if (spi_tool->fd > 0) close(spi_tool->fd);
            free(spi_tool);
      }

}

spi_tool_t *spi_init(char *dev, uint32_t hz)
{
      spi_tool_t *spi_tool = NULL;
      spi_tool = (spi_tool_t*)malloc(sizeof(spi_tool_t));

      if (!spi_tool) return NULL;


      spi_tool->hz = hz;

      spi_tool->xfer[0].tx_buf = 0;
      spi_tool->xfer[0].rx_buf = 0;
      spi_tool->xfer[0].len = 0; // Length of  command to write
      spi_tool->xfer[0].cs_change = 0;
      spi_tool->xfer[0].delay_usecs = 0,
      spi_tool->xfer[0].speed_hz = hz,
      spi_tool->xfer[0].bits_per_word = 8,

      spi_tool->xfer[1].rx_buf = 0;
      spi_tool->xfer[1].tx_buf = 0;
      spi_tool->xfer[1].len = 0; // Length of Data to read
      spi_tool->xfer[1].cs_change = 0;
      spi_tool->xfer[1].delay_usecs = 0;
      spi_tool->xfer[1].speed_hz = hz;
      spi_tool->xfer[1].bits_per_word = 8;


      if ((spi_tool->fd = open(dev, O_RDWR)) < 0)
      {
            spi_deinit(spi_tool);
            return NULL;
      }

      if (ioctl(spi_tool->fd, SPI_IOC_RD_MODE, &spi_tool->mode) < 0)
      {
            spi_deinit(spi_tool);
            return NULL;
      }

      /*
      가능한 mode. spidev.h에 선언되어 있음.

      #define SPI_CPHA        0x01
      #define SPI_CPOL        0x02

      CPOL : Clock Polarity  // 0: high active 1: low active
      CPHA : Clock Phase   // 0: non active -> active로 갈 때 샘플링
                        // 1: active -> non active로 갈 때 샘플링

      #define SPI_MODE_0      (0|0) 
      #define SPI_MODE_1      (0|SPI_CPHA)
      #define SPI_MODE_2      (SPI_CPOL|0)
      #define SPI_MODE_3      (SPI_CPOL|SPI_CPHA)

      #define SPI_CS_HIGH     0x04
      #define SPI_LSB_FIRST   0x08
      #define SPI_3WIRE       0x10
      #define SPI_LOOP        0x20
      #define SPI_NO_CS       0x40
      #define SPI_READY       0x80
      */
      spi_tool->mode = SPI_MODE_0;
      if (ioctl(spi_tool->fd, SPI_IOC_WR_MODE, &spi_tool->mode) < 0)
      {
            spi_deinit(spi_tool);
            return NULL;
      }


      if (ioctl(spi_tool->fd, SPI_IOC_RD_BITS_PER_WORD, &spi_tool->bits) < 0)
      {
            spi_deinit(spi_tool);
            return NULL;
      }

      if (spi_tool->bits != 8)
      {
            spi_tool->bits = 8;
            if (ioctl(spi_tool->fd, SPI_IOC_WR_BITS_PER_WORD, &spi_tool->bits) < 0)
            {
                  spi_deinit(spi_tool);
                  return NULL;
            }
      }

      if (ioctl(spi_tool->fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_tool->hz)<0)
      {
            spi_deinit(spi_tool);
            return NULL;
      }

      return spi_tool;
}

int spi_read(spi_tool_t *spi_tool, char *tx, int tx_len, char *rx, int rx_len)
{
      int status = 0;
      if (!spi_tool || !tx || !rx || spi_tool->fd <= 0)
            return 0;

      spi_tool->xfer[0].tx_buf = (unsigned long)tx;
      spi_tool->xfer[0].len = tx_len;

      spi_tool->xfer[1].rx_buf = (unsigned long)rx;
      spi_tool->xfer[1].len = rx_len;


      status = ioctl(spi_tool->fd, SPI_IOC_MESSAGE(2), spi_tool->xfer);
      if (status < 0)
      {
            return 0;
      }

      return rx_len;
}

int spi_write(spi_tool_t *spi_tool, char *tx, int tx_len)
{
      int status = 0;
      if (!spi_tool || !tx || spi_tool->fd <= 0)
            return 0;

      spi_tool->xfer[0].tx_buf = (unsigned long)tx;
      spi_tool->xfer[0].len = tx_len;

      spi_tool->xfer[1].rx_buf = 0;
      spi_tool->xfer[1].len = 0;


      status = ioctl(spi_tool->fd, SPI_IOC_MESSAGE(1), &spi_tool->xfer[0]);
      if (status < 0)
      {
            return 0;
      }

      return tx_len;
}

int main(int argc, char *argv[])
{
      spi_tool_t *spi_tool = NULL;
      char tx[16] = { 0, };
      char rx[16] = { 0, };

      spi_tool = spi_init("/dev/spidev0.0", 15000000); // 15 Mhz
      if (!spi_tool)
      {
            return 0;
      }

      // example : read chip id
      tx[0] = 0x02; // chip id register address
      tx[1] = 2; // chip id length. 2 bytes
      spi_read(spi_tool, tx, 2, rx, 2);

      // example : write to the register. reg address 0x03
      memset(tx, 0, sizeof(char) * 16);
      tx[0] = 0x03 | 0x80; // must | 0x80
      tx[1] = 2; // data length
      tx[2] = 0x1;
      tx[3] = 0x2;
      spi_write(spi_tool, tx, 4);

      spi_deinit(spi_tool);
     
      return 0;
} 
