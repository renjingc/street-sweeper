 if(i<0)
                    i+=cols;
                else if(i>=cols)
                    i-=cols;
                if(i<20&&i>0)
                {
                    int distRow=maxRow-minRow;
                    if(maxRow<rows/2)
                    {
                        minRow=rows-maxRow;
                        maxRow=rows-minRow;
                    }
                    else
                    {
                        if(minRow<rows/2)
                        {
                            minRow=rows/2;
                            maxRow=rows/2+distRow;
                        }
                    }
                }
                else if(i>160&&i<cols)
                {
                    int distRow=maxRow-minRow;
                    if(minRow>rows/2)
                    {
                        minRow=rows-maxRow;
                        maxRow=rows-minRow;
                    }
                    else
                    {
                        if(maxRow>rows/2)
                        {
                            maxRow=rows/2;
                            minRow=rows/2-distRow;
                        }
                    }
                }
