#pragma once

#pragma pack(push, 1) 
struct  CompressMoveInfo
{
	unsigned short count;       //记录连续相同数据个数
	unsigned short pos;        //有连续相同数据的起始位置

	CompressMoveInfo()
	{
		count = 0;
		pos = 0;
	}
};
#pragma pack(pop)

class CompressInfoCode
{
public:
	CompressInfoCode();
	~CompressInfoCode();

    CompressMoveInfo* infolist;

private:  
    int infolistCount;   //压缩掉数据保存成list的个数 

	float * presrc;      //保存上一帧数据
	int presrcCount;

	float accuracy;      //同上一帧数据做差后保留的精度

public:
	//压缩数据                                             
	inline int Compress(float* src,  int srcCount, float* des, int& desCount)
	{
		CompressMoveInfo cominfo;

        int listCount = 0;
		int usedpos = 0;

		static unsigned long framecount = 0;
		//重新分配空间
		if (srcCount>infolistCount)
		{
			delete[] infolist;
			infolist = NULL;

			infolist = new CompressMoveInfo[srcCount];
			infolistCount = srcCount;
		}


		if (framecount % 30 != 0)
		{
			//循环压缩
			for (int i = 0; i < srcCount; i++)
			{
				//与上一帧数据相同
				if (abs(presrc[i] - src[i]) < accuracy)
				{
					//记录索引
					if (0 == cominfo.count)
					{
						cominfo.pos = i;
					}
					//数目自加
					cominfo.count++;
				}
				else
				{
					if (0 != cominfo.count)
					{
						infolist[listCount++] = cominfo;

						//数据不同，结构体重置
						cominfo.pos = 0;
						cominfo.count = 0;
					}

					//向前压缩
					des[usedpos++] = src[i];
				}
			}
			// 直到循环结束都是一样的数据，应该在循环结束后统计信息保存，返回
			if (0 != cominfo.count)
			{
				infolist[listCount++] = cominfo;
				//数据不同，结构体重置
				cominfo.pos = 0;
				cominfo.count = 0;
			}
		}
		else
		{
			usedpos = srcCount;
			memcpy(des, src, srcCount*sizeof(float));
		}
		
		framecount++;

		//压缩后的des长度返回
		desCount = usedpos;

		//重新分配空间
		if (srcCount != presrcCount)
		{
			delete[] presrc;
			presrc = NULL;

			presrc = new float[srcCount];
			presrcCount = srcCount;
		}

		//重置前一帧数据
		memcpy(presrc, src, srcCount*sizeof(float));

		//返回位置信息的个数
		return listCount;
	}

	//解压数据                              
	inline  float* Uncompress(CompressMoveInfo *srcpos, int infoCount, float* src, int srcCount, float* des, int &desCount)
	{
		int posIndex = 0;
		int srcIndex = 0;
		
		//需要解压的srcpos
		for (int i = 0; i < infoCount; i++)
		{
			// 将跟上一帧不同的拷贝到pos之前，
			if (posIndex < srcpos[i].pos)
			{
				while (posIndex < srcpos[i].pos)
				{
					des[posIndex++] = src[srcIndex++];
				}
			}
			// 将跟上一帧一样的数据拷贝到pos之后，并且拷贝count个
			if (posIndex == srcpos[i].pos)
			{
				while (srcpos[i].count > 0)
				{
					des[posIndex] = presrc[posIndex];
					posIndex++;
					srcpos[i].count--;
				}
			}
		}
		//如果没有压缩，则走这里。否则经过上面的解压后，srcIndex == srcCount
		while (srcIndex<srcCount)
		{
			des[posIndex++] = src[srcIndex++];
		}

		desCount = posIndex;

		//重新分配空间
		if (posIndex != presrcCount)
		{
			delete[] presrc;
			 
			presrc = new float[desCount];
			presrcCount = desCount;
		}
		//更新上一帧数据
		memcpy(presrc, des, desCount*sizeof(float));

		return des;
	}
};

