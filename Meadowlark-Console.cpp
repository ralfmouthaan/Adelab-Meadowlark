// Ralf Mouthaan
// University of Adelaide
// March 2023
//
// Console app to develop class for control of Meadowlark SLM

#include <iostream>
#include <fstream>
#include <string>
#include <Blink_C_wrapper.h>
#include <ImageGen.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class clsMeadowlark {

    protected:

        // Used for creating SDK
        int Board_Number = 1;
        unsigned int Bit_Depth = 12U;
        bool Is_Nematic_Type = true;
        bool Use_GPU = false;
        int Max_Transients = 10U;
        int RAM_Write_Enable = 1;

        // Used for displaying holograms
        bool Wait_For_Trigger = false; // This feature is user - settable; use 1 for 'on' or 0 for 'off'
        int Timeout_ms = 5000;
        bool OutputPulseImageFlip = 0;
        bool OutputPulseImageRefresh = 0; // only supported on 1920x1152, FW rev 1.8.
        char* Regional_LUT;

        // Hadamard hologram-related
        char* HadamardMatrix;
        int HadamardMatrixSize;
        void GenerateHadamardMatrix(int n) {

            // Generates a Hadamard matrix of size n x n

            if (int(log2(n)) - log2(n) != 0) {
                std::cout << "Hadamard matrix size needs to be a multiple of 2";
            }

            if (HadamardMatrixSize >= n) {
                return;
            }

            char* HadamardMatrix = new char[n*n];
            HadamardMatrix[0] = 1;

            for (int x = 1; x < n; x += x) {
                for (int i = 0; i < x; i++) {
                    for (int j = 0; j < x; j++) {

                        HadamardMatrix[(i + x) * n + j] = HadamardMatrix[i * n + j];
                        HadamardMatrix[i * n + j + x] = HadamardMatrix[i * n + j];
                        HadamardMatrix[(i + x) * n + j + x] = -HadamardMatrix[i * n + j];

                    }
                }
            }

            HadamardMatrixSize = n;

        }

    public:

        int Width = 0;
        int Height = 0;
        double WFC_Weights[18] = {};

        clsMeadowlark() {

            unsigned int N_Boards_Found = 0U;
            int Constructed_Okay = true;

            Create_SDK(Bit_Depth, &N_Boards_Found, &Constructed_Okay, Is_Nematic_Type, RAM_Write_Enable, Use_GPU, Max_Transients, Regional_LUT);

            if (Constructed_Okay != 0) {
                Delete_SDK();
                Create_SDK(Bit_Depth, &N_Boards_Found, &Constructed_Okay, Is_Nematic_Type, RAM_Write_Enable, Use_GPU, Max_Transients, Regional_LUT);
            }

            if (Constructed_Okay != 0) {
                std::cout << "SDK Construction Failed";
            }

            Width = Get_image_width(Board_Number);
            Height = Get_image_height(Board_Number);

            LoadLUT("Global");
            LoadWFC();

            
        }
        ~clsMeadowlark() {
            Delete_SDK();
        }
        
        void LoadLUT(const char* LUT) {
            
            // A few special cases
            if (strcmp(LUT, "Global") == 0) {
                LUT = "C:\\Program Files\\Meadowlark Optics\\Blink OverDrive Plus\\LUT Files\\SN5721_WL532_Global.lut";
            }
            else if (strcmp(LUT, "Local") == 0) {
                LUT = "C:\\Program Files\\Meadowlark Optics\\Blink OverDrive Plus\\LUT Files\\SN5721_WL532_Local.txt";
            }
            else if (strcmp(LUT, "Linear") == 0) {
                LUT = "C:\\Program Files\\Meadowlark Optics\\Blink OverDrive Plus\\LUT Files\\12bit_linear.lut";
            }

            Load_LUT_file(Board_Number, (char*)LUT);

        }
        void LoadWFC() {

            std::ifstream reader("C:\\Program Files\\Meadowlark Optics\\Blink OverDrive Plus\\WFC Files\\WFC Zernikes.txt");
            std::string s;
            int i = 0;

            while (std::getline(reader, s)) {
                WFC_Weights[i] = std::stof(s);
                i++;
            }

            reader.close();

            if (i != 19) {
                std::cout << "WFC Zernike text file of incorrect length";
            }
 
        }

        void ShowHologramOnScreen(unsigned char* Holo) {

            cv::Mat img = cv::Mat(Height, Width, CV_8UC1, Holo);
            cv::resize(img, img, cv::Size(), 0.5, 0.5);
            cv::imshow("", img);
            cv::waitKey(0);

        }
        void ShowHologramOnSLM(unsigned char* Holo) {

            Write_image(Board_Number, Holo, Width * Height, Wait_For_Trigger, OutputPulseImageFlip, OutputPulseImageRefresh, Timeout_ms);
            ImageWriteComplete(Board_Number, Timeout_ms);

        }

        unsigned char* ApplyPadding(unsigned char* OriginalHolo, int OriginalHeight, int OriginalWidth) {

            unsigned char* Holo = GenerateBlankHolo();

            int OffsetX = Width / 2 - OriginalWidth / 2;
            int OffsetY = Height / 2 - OriginalHeight / 2;

            for (int i = 0; i < OriginalHeight; i++) {
                for (int j = 0; j < OriginalWidth; j++) {
                    Holo[(i + OffsetY) * Width + j + OffsetX] = OriginalHolo[i*OriginalWidth + j];
                }
            }

            return Holo;

        }
        unsigned char* ApplyZernikes(unsigned char* Holo, double ZernikeWeights[19]) {

            unsigned char* RetVal = new unsigned char[Width * Height];

            // Centered on the SLM, as big as the SLM.
            int CenterX = (int)round(Width / 2);
            int CenterY = (int)round(Height / 2);
            int Radius = std::min(CenterX, CenterY);
            double Piston = 0;

            // User-defined (passed in)
            double TiltX = ZernikeWeights[0];
            double TiltY = ZernikeWeights[1];
            double Defocus = ZernikeWeights[2];
            double AstigX = ZernikeWeights[3];
            double AstigY = ZernikeWeights[4];
            double ComaX = ZernikeWeights[5];
            double ComaY = ZernikeWeights[6];
            double Spherical = ZernikeWeights[7];
            double TrefoilX = ZernikeWeights[8];
            double TrefoilY = ZernikeWeights[9];
            double SecondaryAstigX = ZernikeWeights[10];
            double SecondaryAstigY = ZernikeWeights[11];
            double SecondaryComaX = ZernikeWeights[12];
            double SecondaryComaY = ZernikeWeights[13];
            double SecondarySpherical = ZernikeWeights[14];
            double TetraFoilX = ZernikeWeights[15];
            double TetraFoilY = ZernikeWeights[16];
            double TertiarySpherical = ZernikeWeights[17];
            double QuaternarySpherical = ZernikeWeights[18];

            Generate_Zernike(RetVal, Width, Height, CenterX, CenterY, Radius, Piston, TiltX, TiltY, Defocus, AstigX, AstigY, ComaX, ComaY,
                Spherical, TrefoilX, TrefoilY, SecondaryAstigX, SecondaryAstigY, SecondaryComaX, SecondaryComaY, SecondarySpherical,
                TetraFoilX, TetraFoilY, TertiarySpherical, QuaternarySpherical);

            for (int i = 0; i < Width * Height; i++) {
                RetVal[i] = (unsigned char)((int)RetVal[i] + (int)Holo[i]);
            }

            return RetVal;

        }

        unsigned char* GenerateBinaryGrating(int Gray1, int Gray2, int PixelsPerStripe) {

            unsigned char* Holo = new unsigned char[Width*Height];
            Generate_Stripe(Holo, Width, Height, Gray1, Gray2, PixelsPerStripe);
            return Holo;

        }
        unsigned char* GenerateBlankHolo() {

            unsigned char* Holo = new unsigned char[Width * Height];
            Generate_Solid(Holo, Width, Height, 0);
            return Holo;

        }
        unsigned char* GenerateBlazedGrating(double WeightingX, double WeightingY) {
            
            double ZernikeWeights[19];
            for (int i = 0; i < 19; i++) {
                ZernikeWeights[i] = 0;
            }
            ZernikeWeights[0] = WeightingX;
            ZernikeWeights[1] = WeightingY;
            unsigned char* Holo = GenerateBlankHolo();
            Holo = ApplyZernikes(Holo, ZernikeWeights);
            return Holo;

        }
        unsigned char* GenerateCheckerboard(int Period) {

            unsigned char* Holo = new unsigned char[Width * Height];
            Generate_Checkerboard(Holo, Width, Height, 0, 122, Period);
            return Holo;

        }
        unsigned char* GenerateMacropixels(unsigned char* Array, int ArrayLength, int MacropixelWidth) {

            // Calculate width in macropixels
            int Width_MPx = (int)floor(sqrt((double)ArrayLength));
            int Width_Px = Width_MPx * MacropixelWidth;

            // Error checks
            if (Width_MPx * Width_MPx != ArrayLength) {
                std::cout << "Arraylength must be a square number\n";
            }
            if (Width_MPx * MacropixelWidth > Width) {
                std::cout << "Pattern too large for SLM";
            }
            if (Width_MPx * MacropixelWidth > Height) {
                std::cout << "Pattern too large for SLM";
            }

            unsigned char* Holo = new unsigned char[MacropixelWidth * MacropixelWidth * ArrayLength];

            for (int Macropixeli = 0; Macropixeli < Width_MPx; Macropixeli++) {
                for (int Macropixelj = 0; Macropixelj < Width_MPx; Macropixelj++) {
                    for (int i = 0; i < MacropixelWidth; i++) {
                        for (int j = 0; j < MacropixelWidth; j++) {

                            Holo[(Macropixeli * MacropixelWidth + i) * Width_Px + (Macropixelj * MacropixelWidth + j)] = Array[Macropixeli * Width_MPx + Macropixelj];
                        }
                    }
                }
            }

            Holo = ApplyPadding(Holo, Width_Px, Width_Px);

            return Holo;

        }
        unsigned char* GenerateHadamardMatrix(int ArrayLength, int HadamardIndex, int MacropixelWidth) {

            if (HadamardMatrixSize < ArrayLength) {
                GenerateHadamardMatrix(ArrayLength);
            }

            unsigned char* Array = new unsigned char[ArrayLength];
            for (int i = 0; i < ArrayLength; i++) {
                Array[i] = HadamardMatrix[HadamardIndex * HadamardMatrixSize + i];
            }

            return GenerateMacropixels(Array, ArrayLength, MacropixelWidth);

        }
};

int main()
{
    clsMeadowlark* SLM = new clsMeadowlark;
    
    unsigned char* Holo;
    unsigned char* MacropixelArray = new unsigned char[256];

    for (int i = 0; i < 256; i++) {
        MacropixelArray[i] = i;
    }

    Holo = SLM->GenerateMacropixels(MacropixelArray, 256, 50);
    SLM->ShowHologramOnScreen(Holo);

    return 0;
 
}
