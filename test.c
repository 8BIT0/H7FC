UINT32 znFAT_WriteData(struct FileInfo *pfi, UINT32 len, UINT8 *pbuf)
{
    UINT32 temp = 0, temp1 = 0, len_temp = len;
    UINT32 Cluster_Size = ((pInit_Args->BytesPerSector) * (pInit_Args->SectorsPerClust));

    just_file = pfi;

    if (0 == len)
        return 0; //如果要写入的数据长度为0，则直接返回0

    if (len > (0XFFFFFFFF - pfi->File_Size))
        return (UINT32)-2; //文件大小在写入数据后将发生溢出错误

    znFAT_Seek(pfi, pfi->File_Size); //文件数据定位到文件末尾，文件位置相关信息随即改变

    //检查磁盘剩余空间是否够用
    if ((pfi->File_CurOffset % Cluster_Size) != 0)
    {
        temp = ((pInit_Args->BytesPerSector) - (pfi->File_CurPos)) + ((LAST_SEC_OF_CLU(pfi->File_CurClust)) - (pfi->File_CurSec)) * (Cluster_Size);
        //当前簇剩余数据量

        if (len > temp) //如果要写入的数据量大于temp，则说明必然会超出当前簇，为其扩展空簇
        {
            temp1 = (len - temp) / (Cluster_Size);
            if ((len - temp) % (Cluster_Size))
                temp1++; //计算需要多少个空簇

            if (temp1 > (pInit_Args->Free_nCluster))
                return ((UINT32)-1); //空间不足
        }
    }
    else
    {
        temp1 = len / (Cluster_Size);
        if (len % (Cluster_Size))
            temp1++; //计算需要多少个空簇
        if (temp1 > (pInit_Args->Free_nCluster))
            return ((UINT32)-1); //空间不足
    }

    //===========================================================================================================

    temp = ((pInit_Args->BytesPerSector) - (pfi->File_CurPos)); //计算赋给临时变量，以免后面重复计算

    if ((pfi->File_CurOffset % Cluster_Size) != 0)
    {
        if (len <= temp) //要写入的数据小于等于当前扇区剩余数据量
        {
#ifndef USE_EXCHANGE_BUFFER
            znFAT_Device_Read_Sector(pfi->File_CurSec, znFAT_Buffer);  //读取当前扇区数据，以便作扇区内数据拼接
            Memory_Copy(znFAT_Buffer + pfi->File_CurPos, pbuf, len);   //扇区数据拼接
            znFAT_Device_Write_Sector(pfi->File_CurSec, znFAT_Buffer); //回写扇区数据
#endif

            if (len == temp) //如果要写入的数据正好占满当前扇区
            {
                if (IS_END_SEC_OF_CLU(pfi->File_CurSec, pfi->File_CurClust)) //如果当前扇区是当前簇的最后一个扇区
                {
                    pfi->File_CurSec = SOC(pfi->File_CurClust); //更新当前扇区，其实无效，为了规整
                }
                else //当前扇区不是当前簇的最后扇区
                {
                    pfi->File_CurSec++;
                }

                pfi->File_CurPos = 0;
                pfi->File_CurOffset += len; //更新当前偏移量
                pfi->File_Size += len;      //更新文件大小

#ifdef RT_UPDATE_FILESIZE
                Update_File_Size(pfi); //更文件目录项中的文件大小字段
#endif

                return len;
            }
            else //len小于当前扇区剩余数据量
            {
                //znFAT_Device_Write_Sector(pfi->File_CurSec,ex_buf); //回写扇区数据

                pfi->File_CurPos += (UINT16)len;
                pfi->File_CurOffset += len; //更新当前偏移量
                pfi->File_Size += len;      //更新文件大小

#ifdef RT_UPDATE_FILESIZE
                Update_File_Size(pfi); //更文件目录项中的文件大小字段
#endif

                return len;
            }
        }
        else
        {
            znFAT_Device_Read_Sector(pfi->File_CurSec, znFAT_Buffer);  //读取当前扇区
            Memory_Copy(znFAT_Buffer + pfi->File_CurPos, pbuf, temp);  //扇区数据拼接
            znFAT_Device_Write_Sector(pfi->File_CurSec, znFAT_Buffer); //回写扇区

            len_temp -= temp;
            pbuf += temp;

            if (!(IS_END_SEC_OF_CLU(pfi->File_CurSec, pfi->File_CurClust))) //如果当前扇区不是当前簇的最后一个扇区
            {
                pfi->File_CurSec++;
                pfi->File_CurPos = 0;

                pfi->File_CurOffset += temp;

                temp = (LAST_SEC_OF_CLU(pfi->File_CurClust) - (pfi->File_CurSec) + 1) * (pInit_Args->BytesPerSector); //当前簇中的剩余整整扇区数据量

                if (len_temp <= temp) //如果要写入的数据量小于等于当前簇中的剩余整扇区数据量
                {
                    temp1 = len_temp / (pInit_Args->BytesPerSector); //计算要写入的数据量够几个整扇区

                    znFAT_Device_Write_nSector(temp1, pfi->File_CurSec, pbuf);
                    pbuf += ((pInit_Args->BytesPerSector) * temp1);

                    if (len_temp == temp) //如果正好写满当前簇
                    {
                        pfi->File_CurSec = SOC(pfi->File_CurClust); //囧簇
                        pfi->File_CurPos = 0;

                        pfi->File_CurOffset += len_temp;
                        pfi->File_Size += len;

#ifdef RT_UPDATE_FILESIZE
                        Update_File_Size(pfi); //更文件目录项中的文件大小字段
#endif

                        return len;
                    }
                    else
                    {
                        pfi->File_CurSec += temp1;
                        pfi->File_CurPos = (UINT16)(len_temp % (pInit_Args->BytesPerSector));

                        if (pfi->File_CurPos) //还有要写的数据,处理最后的字节数据
                        {
                            Memory_Copy(znFAT_Buffer, pbuf, pfi->File_CurPos);
                            znFAT_Device_Write_Sector(pfi->File_CurSec, znFAT_Buffer);

                            pfi->File_CurOffset += len_temp;
                            pfi->File_Size += len;

#ifdef RT_UPDATE_FILESIZE
                            Update_File_Size(pfi); //更文件目录项中的文件大小字段
#endif

                            return len;
                        }
                    }
                    else
                    {
                        temp1 = temp / (pInit_Args->BytesPerSector);

                        znFAT_Device_Write_nSector(temp1, pfi->File_CurSec, pbuf);
                        pbuf += temp;

                        len_temp -= temp;

                        pfi->File_CurSec = SOC(pfi->File_CurClust);
                        pfi->File_CurPos = 0;

                        pfi->File_CurOffset += temp;
                    }
                }
                else //当前扇区是当前簇最后一个扇区
                {
                    pfi->File_CurSec = SOC(pfi->File_CurClust);
                    pfi->File_CurPos = 0;

                    pfi->File_CurOffset += temp;
                }
            }
        }
    }

    UINT8 znFAT_Seek(struct FileInfo * pfi, UINT32 offset)
    {
        UINT32 Cluster_Size = ((pInit_Args->SectorsPerClust) * (pInit_Args->BytesPerSector)); //计算簇的总字节数据，以免后面重复计算
        UINT32 temp = 0, temp1 = 0, temp2 = 0, len = 0, k = 0, ncluster = 0, have_read = 0;

        just_file = pfi;

#ifndef RT_UPDATE_CLUSTER_CHAIN
        get_next_cluster_in_cccb = 1;
#ifdef USE_ALONE_CCCB
        CCCB_To_Alone();
#endif
#endif

        if (offset < (pfi->File_Size)) //如果要定位到的偏移量小于文件大小，则必定不在文件末尾
        {
            pfi->File_IsEOF = BOOL_FALSE;
        }

        if (offset == (pfi->File_CurOffset))
            return 0; //如果要定位的位置正好是当前偏移量，则直接返回

        if (offset < (pfi->File_CurOffset)) //如果要定位的位置在当前偏移量之前，则先回到文件起点，因为簇链是单向的
        {
            pfi->File_CurClust = pfi->File_StartClust;
            pfi->File_CurSec = SOC(pfi->File_CurClust);
            pfi->File_CurPos = 0;
            pfi->File_CurOffset = 0;
            pfi->File_IsEOF = BOOL_FALSE;
        }

        len = offset - (pfi->File_CurOffset); //计算目标偏移量到当前偏移量之间的数据长度

        if (offset >= (pfi->File_Size)) //如果从当前位置开始要读取的数据长度len不小于文件大小
        {
            len = (pfi->File_Size - pfi->File_CurOffset); //对len进行修正，置为文件剩余可读数据量。
            pfi->File_IsEOF = BOOL_TRUE;                  //这种情况下，文件必然会读到末尾。
        }

        //===================================================================
        if ((pfi->File_CurOffset % Cluster_Size) != 0) //如果当前偏移量是簇大小整数倍，说明此位置即为整簇开始
        {                                              //不要再进行当前簇内数据处理，直接进入簇-扇区-字节阶段
            if (len <= (pInit_Args->BytesPerSector - pfi->File_CurPos))
            {
                //更新当前位置参数
                if ((pInit_Args->BytesPerSector - pfi->File_CurPos) == len) //如果正好读到当前扇区的末尾
                {
                    if (IS_END_SEC_OF_CLU(pfi->File_CurSec, pfi->File_CurClust)) //如果当前扇区是当前簇的最后一个扇区
                    {
                        if (!pfi->File_IsEOF)
                        {
                            pfi->File_CurClust = Get_Next_Cluster(pfi->File_CurClust);
                        }
                        pfi->File_CurSec = SOC(pfi->File_CurClust);
                    }
                    else
                    {
                        pfi->File_CurSec++;
                    }
                    pfi->File_CurPos = 0;
                }
                else
                {
                    pfi->File_CurPos += ((UINT16)len);
                }
                pfi->File_CurOffset += len;

                return NUL_RET;
            }
            //===================================================================
            else
            {
                temp = (pInit_Args->BytesPerSector - pfi->File_CurPos); //将当前扇区的剩余数据量赋给中间变量temp
                have_read += temp;

                if (!(IS_END_SEC_OF_CLU(pfi->File_CurSec, pfi->File_CurClust))) //如果当前扇区不是当前簇的最后一个扇区
                {
                    pfi->File_CurSec++;
                    pfi->File_CurPos = 0;

                    temp2 = (len - have_read);                                                                               //计算剩余数据量
                    temp1 = ((LAST_SEC_OF_CLU(pfi->File_CurClust) - (pfi->File_CurSec - 1)) * (pInit_Args->BytesPerSector)); //剩余所有扇区数据量
                    if (temp2 <= temp1)                                                                                      //如果剩余数据量xxx
                    {
                        //这说明要读的数据在当前簇内，没有跨到下一簇
                        temp = temp2 / (pInit_Args->BytesPerSector); //计算当前簇内整扇区读取的结束扇区
                        have_read += ((pInit_Args->BytesPerSector) * temp);

                        if (temp2 == temp1)
                        {
                            if (!pfi->File_IsEOF)
                            {
                                pfi->File_CurClust = Get_Next_Cluster(pfi->File_CurClust);
                            }
                            pfi->File_CurSec = SOC(pfi->File_CurClust);
                            pfi->File_CurPos = 0;
                        }
                        else
                        {
                            pfi->File_CurSec += temp;
                            //更新当前位置参数
                            pfi->File_CurPos = (UINT16)(len - have_read);
                        }
                        pfi->File_CurOffset += len;

                        return NUL_RET;
                    }
                    else //如果剩余数据的整扇区数不小于当前簇的剩余扇区数，即要读的数据不光在当前簇内，已经跨簇了
                    {
                        temp = LAST_SEC_OF_CLU(pfi->File_CurClust) - (pfi->File_CurSec) + 1; //计算当前簇的剩余整扇区数
                        have_read += ((pInit_Args->BytesPerSector) * temp);
                    }
                }

                //更新当前位置参数，此时已经读完当前簇的所有剩余数据，跨到下一簇
                pfi->File_CurClust = Get_Next_Cluster(pfi->File_CurClust);
                pfi->File_CurSec = SOC(pfi->File_CurClust);
                pfi->File_CurPos = 0;
            }
        }
        //----------------------------以上是处理当前簇内的数据-------------------------------------
        if (len - have_read > 0)
        {
            ncluster = (len - have_read) / Cluster_Size; //计算剩余数据的整簇数

            //更新当前位置参数，此时已经读完所有的整簇数据

            for (k = 0; k < ncluster; k++) //读取整簇数据
            {
                have_read += (Cluster_Size);
                if (!((len - have_read) == 0 && pfi->File_IsEOF))
                {
                    pfi->File_CurClust = Get_Next_Cluster(pfi->File_CurClust);
                }
            }

            pfi->File_CurSec = SOC(pfi->File_CurClust);

            //----------------------------以上是处理整簇数据------------------------------------------
            if (len - have_read > 0)
            {
                temp = (len - have_read) / (pInit_Args->BytesPerSector); //计算最终剩余数据的整扇区数
                have_read += ((pInit_Args->BytesPerSector) * temp);

                pfi->File_CurSec += temp;
                //----------------------------以上是处理整扇区数据----------------------------------------
                if (len - have_read > 0)
                {
                    //更新当前位置参数，此时数据读取操作已经结束
                    pfi->File_CurPos = (UINT16)(len - have_read);
                }
                //----------------------------以上是处理最后扇区内的剩余字节----------------------------------------
            }
        }

        pfi->File_CurOffset += len;

        return 0;
    }
