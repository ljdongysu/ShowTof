#include <opencv2/core.hpp>
#include "SpeckleFilter3d.h"
void SpeckleFileter3d(cv::Mat3f& src, cv::Scalar newVal, int areaThred, float thredPtr, int axis)//cv::Mat &depth_thred_mat)
    {
        int  h = src.rows;
        int  w = src.cols;
        int  pixels = h * w;
        bool* hasLabeled = new bool[pixels];
        int* connectX = new int[pixels];
        int* connectY = new int[pixels];
        int* recordLoc = new int[pixels];

        float* srcPtr = (float*)src.data;
        //float* thred_ptr = (float*)depth_thred_mat.data;

        int hsize = 3 * w;
        int loci = 0;
        int h1 = h - 1;
        int w1 = w - 1;

        for (int k = 0; k < pixels; k++)
        {
            hasLabeled[k] = false;
        }

        for (int i = 0; i < h; i++)
        {
            for (int j = 0; j < w; j++)
            {
                int loc = loci + j;
                int loc3Z = loc * 3 + axis;

                if (!hasLabeled[loc])
                {
                    int connectNum = 0;
                    int recordNum = 0;

                    connectX[connectNum] = j;
                    connectY[connectNum] = i;

                    hasLabeled[loc] = true;

                    while (connectNum >= 0 && srcPtr[loc3Z] > 0)
                    {
                        int curX = connectX[connectNum];
                        int curY = connectY[connectNum];

                        int curLoc = curX + curY * w;
                        int curLocZ = curLoc * 3 + axis;
                        float curValue = srcPtr[curLocZ];
                        float depthThred;// = fmin(0.1,fmax(0.002,thred_ptr * curValue));;//thred_ptr;//*curValue;//[curLoc];

                        // 可以在外面并行算
                        float dis2 = powf(srcPtr[curLoc * 3], 2) + powf(srcPtr[curLoc * 3 + 1], 2) + powf(srcPtr[curLoc * 3 + 2], 2);
                        depthThred = thredPtr * dis2;

                        recordLoc[recordNum] = curLocZ;

                        connectNum -= 1;
                        recordNum += 1;

                        //左边
                        if (curX > 1 && (!hasLabeled[curLoc - 1]) && fabs(srcPtr[curLocZ - 3] - curValue) < depthThred)
                        {
                            connectNum += 1;
                            connectX[connectNum] = curX - 1;
                            connectY[connectNum] = curY;
                            hasLabeled[curLoc - 1] = true;
                        }
                        //右边
                        if (curX < w1 && (!hasLabeled[curLoc + 1]) && fabs(srcPtr[curLocZ + 3] - curValue) < depthThred)
                        {
                            connectNum += 1;
                            connectX[connectNum] = curX + 1;
                            connectY[connectNum] = curY;
                            hasLabeled[curLoc + 1] = true;
                        }
                        //上边
                        if (curY > 1 && (!hasLabeled[curLoc - w]) && fabs(srcPtr[curLocZ - hsize] - curValue) < depthThred)
                        {
                            connectNum += 1;
                            connectX[connectNum] = curX;
                            connectY[connectNum] = curY - 1;
                            hasLabeled[curLoc - w] = true;
                        }
                        //下边
                        if (curY < h1 && (!hasLabeled[curLoc + w]) && fabs(srcPtr[curLocZ + hsize] - curValue) < depthThred)
                        {
                            connectNum += 1;
                            connectX[connectNum] = curX;
                            connectY[connectNum] = curY + 1;
                            hasLabeled[curLoc + w] = true;
                        }
                    }

                    //判断面积
                    if (recordNum < areaThred && srcPtr[loc3Z] > 0)
                    {
                        for (int k = 0; k < recordNum; k++)
                        {
                            int cur_pos = recordLoc[k] - axis;
                            srcPtr[cur_pos] = newVal[0];
                            srcPtr[cur_pos + 1] = newVal[1];
                            srcPtr[cur_pos + 2] = newVal[2];
                        }
                    }
                }
            }

            loci += w;
        }


        delete[] hasLabeled;
        delete[] connectX;
        delete[] connectY;
        delete[] recordLoc;
    }
