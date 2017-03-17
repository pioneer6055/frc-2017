/*
 * Pixy.cpp
 *
 *  Created on: Oct 20, 2016
 *      Author: Chester Marshall, mentor for 6055 and 5721
 *
 *  Code adapted from github repository of RoboDox Team #599
 *  https://github.com/Robodox-599/Pixy
 *
 *
 */


#include "Pixy.h"

Pixy::Pixy()
{
	i2c = new I2C(I2C::Port::kOnboard, 0x54); //(I2C::Port::kOnboard or kMXP, Pixy Address)
	blockType = NORMAL_BLOCK;// it is the enum on the top
	skipStart = false;	//skips to check 0xaa55, which is byte that tells pixy it is start of new frame
	blockCount = 0; //How many signatured objects are there?t array
}

Pixy::~Pixy()
{
	delete i2c;
	i2c = nullptr;
}

void Pixy::Block::print()
{
	int i, j;
	char sig[6], d;
	bool flag;
	if (signature>PIXY_MAX_SIGNATURE) // color code! (CC)
	{
	  // convert signature number to an octal string
	  for (i=12, j=0, flag=false; i>=0; i-=3) //assigns value to signature, x, y, width, height, and anlge
	  {
		d = (signature>>i)&0x07;
		if (d>0 && !flag)
		  flag = true;
		if (flag)
		  sig[j++] = d + '0';
	  }
	  sig[j] = '\0';
	  printf("CC block! sig: %s (%d decimal) x: %d y: %d width: %d height: %d angle %d\n", sig, signature, x, y, width, height, angle);
	}
	else // regular block.  Note, angle is always zero, so no need to print
	  printf("sig: %d x: %d y: %d width: %d height: %d\n", signature, x, y, width, height); //prints out data to console instead of smartDashboard -> check on the side of the driver station, check +print and click view console
}

bool Pixy::getStart()
{
	uint16_t w, lastw;

	lastw = 0xffff;

	while(true)
	{
		w = getWord();
		if (w==0 && lastw==0)
		{
		  //delaySeconds(0.01);
		  return false;
		}
		else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD)
		{
		  blockType = NORMAL_BLOCK;
		  return true;
		}
		else if (w==PIXY_START_WORD_CC && lastw==PIXY_START_WORD)
		{
		  blockType = CC_BLOCK;
		  return true;
		}
		else if (w==PIXY_START_WORDX) //when byte recieved was 0x55aa instead of otherway around, the code syncs the byte
		{
		  printf("Pixy: reorder");
		  getByte(); // resync
		}
		lastw = w;
	}

}

uint16_t Pixy::getWord()
{
	unsigned char buffer[2] = {0, 0};

	i2c->ReadOnly(2, buffer);
	return (buffer[1] << 8) | buffer[0]; //shift buffer[1] by 8 bits and add( | is bitwise or) buffer[0] to it
}

uint8_t Pixy::getByte()
{
	unsigned char buffer[1] = {0};

	i2c->ReadOnly(1, buffer);
	return buffer[0];
}

void Pixy::setBrightness(unsigned char brightness)
{
	//brightness = 0-255
	i2c->Write(PIXY_CAM_BRIGHTNESS_SYNC,brightness);
}

//this is the main processing function - called from robot.cpp
uint16_t Pixy::getBlocks(uint16_t maxBlocks)
{
	blocks[0] = {0}; //resets the array - clears out data from previous reading
	uint8_t i;
	uint16_t w, checksum, sum;
	Block *block;

	if(!skipStart) //when computer has not seen 0xaa55 (starting frame)
	{
		if (getStart()==false)
		{
			return 0;
		}
	}
	else
	skipStart = false;

	for(blockCount=0; blockCount<maxBlocks && blockCount<PIXY_MAXIMUM_ARRAYSIZE;)
	{
		checksum = getWord();
		if (checksum==PIXY_START_WORD) // we've reached the beginning of the next frame - checking for 0xaa55
		{
			skipStart = true; //starts this function
			blockType = NORMAL_BLOCK;
		    return blockCount;
		}
		else if (checksum==PIXY_START_WORD_CC) //we've reached the beginning of the next frame - checking for 0xaa56
		{
			skipStart = true;
			blockType = CC_BLOCK;
			return blockCount;
		}
		else if (checksum==0)
		{
		    return blockCount;
		}

		block = blocks + blockCount;

		for (i=0, sum=0; i<sizeof(Block)/sizeof(uint16_t); i++)
		{
			if (blockType==NORMAL_BLOCK && i>=5) // skip --if not an CC block, no need to consider angle
			{
				block->angle = 0;
				break;
			}
		    w = getWord();
		    sum += w; //sum = w + sum
		    *((uint16_t *)block + i) = w; //converts block to integer value
		}

		if (checksum==sum)
		{
			blockCount++;
		}
		else
		{
			printf("Pixy: cs error");
		}

		w = getWord(); //when this is start of the frame
		if (w==PIXY_START_WORD)
		{
			blockType = NORMAL_BLOCK;
		}
		else if (w==PIXY_START_WORD_CC)
		{
			blockType = CC_BLOCK;
		}
		else
		{
		    return blockCount;
		}
	}
	return 0;
}

