#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <string>

class SynImage
{
public:
	//constructor
	SynImage() :
		arrImages(NULL),
		nRows(0),
		nCols(0),
		nFrames(0)
	{
	};

	//constructor
	explicit SynImage(uint32_t Rows, uint32_t Cols)
		:arrImages(NULL)
	{
		// validate params
		// check if Rows, Cols and Frames > 0
		this->nRows = Rows;
		this->nCols = Cols;
		this->nFrames = 1;

		uint32_t imageSize = Rows * Cols;
		if (imageSize > 0)
		{
			arrImages = new int16_t[imageSize];
			memset(arrImages, 0, sizeof(int16_t)*imageSize);
		}
	};

	//constructor
	explicit SynImage(uint32_t Rows, uint32_t Cols, uint32_t Frames)
		:arrImages(NULL)
	{
		// validate params
		// check if Rows, Cols and Frames >0

		this->nRows = Rows;
		this->nCols = Cols;
		this->nFrames = Frames;

		uint32_t imageSize = Rows * Cols * Frames;
		if (imageSize > 0)
		{
			arrImages = new int16_t[imageSize];
			memset(arrImages, 0, sizeof(int16_t)*imageSize);
		}
	};

	//desctructor
	~SynImage()
	{
		if (arrImages != NULL)
		{
			delete[] arrImages;
			arrImages = NULL;
		}
	};

	//copy constructor
	SynImage(const SynImage &image)
	{
		//validate params
		this->nRows = image.nRows;
		this->nCols = image.nCols;
		this->nFrames = image.nFrames;

		uint32_t imageSize = nRows * nCols * nFrames;
        this->arrImages = new int16_t[imageSize];
		memcpy(arrImages, image.arrImages, sizeof(int16_t)*imageSize);

	};

	//copy assignment of opertor =
	const SynImage & operator =(const SynImage & image)
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
			if (NULL != arrImages)
			{
				delete[] arrImages;
			}

			arrImages = new int16_t[imageSize];
			memcpy(arrImages, image.arrImages, sizeof(int16_t)*imageSize);
		}

		return *this;
	};


	//move constructor
	SynImage(SynImage&& image)
		:arrImages(nullptr)
		, nRows(0)
		, nCols(0)
		, nFrames(0)
	{
		*this = std::move(image);
	};

	//move assignment
	SynImage& operator =(SynImage&& image)
	{
		if (this == &image)
			return *this;

		this->nRows = image.nRows;
		this->nCols = image.nCols;
		this->nFrames = image.nFrames;

		if (this->arrImages != nullptr)
		{
			delete[] arrImages;
		}
		arrImages = image.arrImages;

		image.arrImages = nullptr;
		image.nRows = 0;
		image.nCols = 0;
		image.nFrames = 0;

		return *this;
	};

	uint32_t Clear()
	{
		if (arrImages != NULL)
		{
			uint32_t imageSize = nRows * nCols * nFrames;
			memset(arrImages, 0, imageSize * sizeof(int16_t));
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
			if (arrImages == NULL)
			{
				arrImages = new int16_t[imageSize];
			}
			else
			{
				delete[] arrImages;
				arrImages = new int16_t[imageSize];
			}

			memset(arrImages, 0, sizeof(int16_t)* imageSize);
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
			fwrite(arrImages, sizeof(int16_t), nRows*nCols, fileImage);
			fclose(fileImage);
		}
	}

	int16_t *arrImages;	// n frames
	uint32_t nRows;
	uint32_t nCols;
	uint32_t nFrames;
};
