#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#include "cmlLogging.h"

typedef uint16_t pixel_t;

class CPImage
{
public:
	//constructor
	CPImage()
		:arrImages(nullptr)
		,nRows(0)
		,nCols(0)
		,nFrames(0)
	{
	};

	//constructor
	explicit CPImage(uint32_t Rows, uint32_t Cols) 
		:arrImages(nullptr)
	{
		// validate params
		// check if Rows, Cols and Frames >0
		
		this->nRows = Rows;
		this->nCols = Cols;
		this->nFrames = 1;

		uint32_t imageSize = Rows * Cols;

		if (imageSize > 0)
		{
			arrImages = new pixel_t[imageSize];
			memset(arrImages, 0, sizeof(pixel_t)*imageSize);
		}
		
		cmlLogging::GetLogger()->Log("Construct New Image: Row%d Col%d", Rows, Cols);
	};

	//constructor
	explicit CPImage(uint32_t Rows, uint32_t Cols, uint32_t Frames) 
		:arrImages(nullptr)
	{
		// validate params
		// check if Rows, Cols and Frames >0
		
		this->nRows = Rows;
		this->nCols = Cols;
		this->nFrames = Frames;

		uint32_t imageSize = Rows * Cols * Frames;

		if (imageSize > 0)
		{
			arrImages = new pixel_t[imageSize];
			memset(arrImages, 0, sizeof(pixel_t)*imageSize);
		}

		cmlLogging::GetLogger()->Log("Construct New Image: Row%d Col%d Frames%d", Rows, Cols, Frames);
	};

	//desctructor
	~CPImage()
	{
		//cmlLogging::GetLogger()->Log("~CPImage(), no destrution");

		if (arrImages != nullptr)
		{
			delete[] arrImages;
			arrImages =nullptr;
			cmlLogging::GetLogger()->Log("Destruct Image.");
		}
	};

	//copy constructor
	CPImage(const CPImage& image)
	{
		//validate params
		this->nRows = image.nRows;
		this->nCols = image.nCols;
		this->nFrames = image.nFrames;
		uint32_t imageSize = nRows * nCols * nFrames;
		this->arrImages = new pixel_t[imageSize];

		memcpy(arrImages, image.arrImages, sizeof(pixel_t)*imageSize);
		//std::copy(image.arrImages, image.arrImages+sizeof(pixel_t)*imageSize, this->arrImages);

		cmlLogging::GetLogger()->Log("Copy Constructor: Row%d Col%d Frames%d", image.nRows, image.nCols, image.nFrames);
	}

	//copy assignment of opertor =
	CPImage & operator =(const CPImage& image)
	{
		if (this == &image) 
			return *this;

		this->nRows = image.nRows;
		this->nCols = image.nCols;
		this->nFrames = image.nFrames;
		//this->arrImages = image.arrImages;
		uint32_t imageSize = this->nRows * this->nCols * this->nFrames;

		if (imageSize > 0)
		{
			if (nullptr == this->arrImages)
			{
				this->arrImages = new pixel_t[imageSize];
			}
			else
			{
				delete[] (this->arrImages);
				this->arrImages = new pixel_t[imageSize];
			}

			memcpy((this->arrImages), image.arrImages, sizeof(pixel_t)*imageSize);
			//std::copy(image.arrImages, image.arrImages+sizeof(pixel_t)*imageSize, this->arrImages);
		}
		cmlLogging::GetLogger()->Log("Copy Assignment: Row%d Col%d Frames%d", image.nRows, image.nCols, image.nFrames);

		return *this;
	};


#if 1
	//move constructor
	CPImage(CPImage&& image)
		:arrImages(nullptr)
		, nRows(0)
		, nCols(0)
		, nFrames(0)
	{
		*this = std::move(image);
		/*
			std::move stand for:

			this->arrImages = image.arrImages;
			this->nRows = image.nRows;
			this->nCols = image.nCols;
			this->nFrames = image.nFrames;

			image.arrImages = nullptr;
			image.nRows = 0;
			image.nCols = 0;
			image.nFrames = 0;
		*/

		cmlLogging::GetLogger()->Log("Move Constructor: Row%d Col%d Frames%d", this->nRows, this->nCols, this->nFrames);
	}


	//move assignment
	CPImage& operator =(CPImage&& image)
	{
		if (this == &image) 
			return *this;

		this->nRows = image.nRows;
		this->nCols = image.nCols;
		this->nFrames = image.nFrames;

		if (this->arrImages != nullptr)
		{
			delete[] this->arrImages;
		}
		this->arrImages = image.arrImages;
		
		image.arrImages = nullptr;
		image.nRows = 0;
		image.nCols = 0;
		image.nFrames = 0;

		cmlLogging::GetLogger()->Log("Move Assignment: Row%d Col%d Frames%d", this->nRows, this->nCols, this->nFrames);
		return *this;
	}

#endif

	uint32_t Clear()
	{
		if (arrImages != nullptr)
		{
			uint32_t imageSize = nRows * nCols * nFrames;
			memset(arrImages, 0, imageSize * sizeof(pixel_t));
		}
	}

	//resize
	uint32_t ReAlloc(uint32_t Rows, uint32_t Cols, uint32_t Frames = 1)
	{
		//validate params
		this->nRows = Rows;
		this->nCols = Cols;
		this->nFrames = Frames;

		uint32_t imageSize = Rows * Cols * Frames;

		if (imageSize > 0)
		{
			if (arrImages == nullptr)
			{
				arrImages = new pixel_t[imageSize];
				cmlLogging::GetLogger()->Log("Alloc New Image: Rows%d, Cols%d, Frames%d", Rows, Cols, Frames);
			}
			else
			{
				delete[] arrImages;
				arrImages = new pixel_t[imageSize];
				cmlLogging::GetLogger()->Log("ReAlloc New Image: Rows%d, Cols%d, Frames%d", Rows, Cols, Frames);
			}

			memset(arrImages, 0, sizeof(pixel_t) * imageSize);

		}

		return 0;
	}

	uint32_t Size()
	{
		return nRows*nCols*nFrames;
	}

	std::string ToString(std::string strDivideLabel = ",")
	{
		std::string strInfo;
		if (0 != nRows && 0 != nCols)
		{
			uint32_t k = 0;
			for (uint32_t i = 0; i < nRows; i++)
			{
				for (uint32_t j = 0; j < nCols; j++)
				{
					if (j == (nCols - 1))
						strInfo += std::to_string(arrImages[k]);
					else
						strInfo += std::to_string(arrImages[k]) + strDivideLabel;

					k++;
				}
				strInfo += "\n";
			}
		}

		return strInfo;
	}

	void LogRaw(std::string strPrefix)//strPrefix include save path
	{
		std::string strFile = strPrefix + std::to_string(nCols) + "x" + std::to_string(nRows) + ".raw";
		FILE *fileImage = NULL;
		if (fopen_s(&fileImage, strFile.c_str(), "wb") == 0)
		{
			fwrite(arrImages, sizeof(pixel_t), nRows*nCols, fileImage);
			fclose(fileImage);
		}
	}

	pixel_t *arrImages;	// n frames
	uint32_t nRows;
	uint32_t nCols;
	uint32_t nFrames;
};
