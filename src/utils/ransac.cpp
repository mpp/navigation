#include "ransac.h"


// conversion to string for many types
template <class T>
string toString(T a) {
    stringstream ss (stringstream::in | stringstream::out);
    ss << a;
    return(ss.str());
}

// outputs redctangle that includes the data
cv::Rect dataRange(float* x, float* y, const int N) {

    float minx = x[0];
    float miny = y[0];
    float maxx = x[0];
    float maxy = y[0];

    for (int i=1; i<N; i++) {
        if (minx > x[i])
            minx = x[i];
        if (miny > y[i])
            miny = y[i];
        if (maxx < x[i])
            maxx = x[i];
        if (maxy < y[i])
            maxy = y[i];
    }
    Rect_<float> rect(minx, miny, maxx-minx+1, maxy-miny+1);

    return rect;
}

// simple linear transformation
inline float linear(float src, float mult_val, float plus_val) {
    return(src*mult_val+plus_val);
}

// shows points, marks inliers, draws a fit line (Y points up);
void showPoints(Mat& image, float* x, float* y, const int N,
        bool* inlrs, float* param) {

    bool* inliers = inlrs;
    if (inlrs==NULL) {
        inliers = new bool[N];
        std::fill_n(inliers, N, true);
    }

    // image dimensions
    int h = image.rows;
    int w = image.cols;
    const int pad = 10; // border padding

    // range of data
    Rect_<float> range  = dataRange(x, y, N);
    string str = "x: " + toString(range.x) + ".." + toString(range.x+range.width) +
            "; y: " + toString(range.y) + ".." + toString(range.y+range.height) +
            "; data: y = " + toString(TRUE_PARAM_UNIT_TEST[0]) + "x + " +
            toString(TRUE_PARAM_UNIT_TEST[1]) + " + noise";
    putText(image, str, Point(20, 20), FONT_HERSHEY_PLAIN, 1, Scalar(255) );

    // maping data to image
    float dataToImg = min((h-2*pad)/range.height,
            (w-2*pad)/range.width);

    // inverse y direction
    int x0 = pad;
    int y0 = h-pad;

    // points
    for (int i=0; i<N; i++) {

        // adjust for origin
        float datax = x[i]-range.x;
        float datay = y[i]-range.y;

        // transform to image coordiantes
        int imgx = linear(datax, dataToImg, x0) + 0.5f;
        int imgy = linear(datay, -dataToImg, y0) + 0.5f;

        int thickness = inliers[i]?2:1;
        circle(image, Point(imgx, imgy), 2, Scalar(255), thickness);
    }

    // line points
    float datax1 = range.x;
    float datax2 = range.x+range.width-1;
    float datay1 = linear(datax1, param[0], param[1]);
    float datay2 = linear(datax2, param[0], param[1]);

    // adjust for origin
    datax1-=range.x;
    datax2-=range.x;
    datay1-=range.y;
    datay2-=range.y;

    // transform to image coordiantes
    int x1 = linear(datax1, dataToImg, x0) + 0.5f;
    int x2 = linear(datax2, dataToImg, x0) + 0.5f;
    int y1 = linear(datay1, -dataToImg, y0) + 0.5f; // Y points up
    int y2 = linear(datay2, -dataToImg, y0) + 0.5f;

    cv::line(image, Point(x1, y1), Point(x2, y2), Scalar(255), 1);

    if (inlrs==NULL)
        delete inliers;
}

// Root mean sqaure error (RMSE) of line fit y=param[0]*x+param[1]
float errorLine(float* x, float* y, int N, float* param,
        const bool* inliers) {

    if (N<=0 || x==NULL || y==NULL || param==NULL)
        return MAX_FLOAT;

    int ninliers = (inliers==NULL?N:0);
    double RMSE = 0;

    // error accumulation loop
    for (int i = 0; i < N; i++) {

        if (inliers!=NULL) {
            if (inliers[i])
                ninliers++;
            else
                continue;
        }

        float y_predicted = param[0] * x[i] + param[1];
        RMSE += SQR(y[i] - y_predicted);
    }

    if (ninliers==0)
        return LARGE_NUMBER;
    else
        return sqrt(RMSE/ninliers);
}

// simple line fit using sum of squared differences
// inliers are used to specify a subset of points for a fit
float fitLine(float* x, float* y, int N, float* param,
        const bool* inliers, const bool debug) {

    if (N<=0 || x==NULL || y==NULL || param==NULL) {
        if (debug)
            cout<<"ERROR fit line quadratic: N<=0 || x==NULL || y==NULL || param==NULL"<<endl;
        return MAX_FLOAT;
    }

    int ninliers = (inliers==NULL?N:0);
    double sum_x = 0;
    double sum_y = 0;
    double sum_xy = 0;
    double sum_x2 = 0;

    // create specific sums of x, y, xy, x^2
    for (int i = 0; i < N; i++) {

        // use inliers only
        if (inliers!=NULL) {
            if (!inliers[i])
                continue;
            else
                ninliers++;
        }

        sum_x += x[i];
        sum_y += y[i];
        sum_xy += x[i] * y[i];
        sum_x2 += x[i] * x[i];
    }

    if(ninliers < 2) {
        if (debug)
            cout<<"ERROR fit line quadratic: less than 2 data points"<<endl;
        return MAX_FLOAT;
    }

    // means
    double mean_x = sum_x / ninliers;
    double mean_y = sum_y / ninliers;

    float varx = sum_x2 - sum_x * mean_x;
    float cov = sum_xy - sum_x * mean_y;
    // eliminate bias: variance is e a bit underestimated since df=N-1, see
    // http://davidmlane.com/hyperstat/B16616.html)
//  if (ninliers>1)
//      varx *= (float)ninliers/(ninliers-1);


    // quadratic fit
    if (abs(varx) < SMALL_NUMBER) {
        if (debug)
            cout<<"ERROR fit line quadratic: zero variance" <<endl;
        return MAX_FLOAT;
    }

    // see http://easycalculation.com/statistics/learn-regression.php
    param[0] = cov / varx;
    param[1] = mean_y - param[0] * mean_x;

    return errorLine(x, y, N, param, inliers);
}

// RANSAC line fit (number of inliers has priority over RMSE).
float RANSAC_line(float* x, float* y, const int N, float* param,
        const int niter, const float maxError,
        bool* inlrs, bool debug) {

    if (x==NULL || y==NULL || N<2) {
        if (debug)
            cout<<"x==NULL || y==NULL || N<2" <<endl;
        return MAX_FLOAT;
    }
    srand (time(NULL));

    // internal stopping criterions
    const float RMSE_OK = 0.01f;
    const float INLIERS_RATIO_OK = 0.9f;

    int ninliers, best_ninliers = 0;
    float inliers_ratio = 0;
    float RMSE, bestRMSE = MAX_FLOAT;
    float cur_param[2];

    bool* inliers = inlrs;
    if (inlrs==NULL) {
        inliers = new bool[N];
        std::fill_n(inliers, N, true);
    }

    // iterations
    int iter;
    for (iter = 0; iter<niter; iter++) {

        // 1. select a random set of 2 inliers
        int i1 = rand() % N; // [0, N[
        int i2 = i1;
        while (i2==i1)
            i2 = rand() % N;

        // 2. select minimum number of points (2)
        float x1 = x[i1];
        float x2 = x[i2];
        float y1 = y[i1];
        float y2 = y[i2];

        // TODO: we may parameterize the line differently (alpha, d)
        if (abs(x1-x2) < SMALL_NUMBER)
            cur_param[0] = LARGE_NUMBER * SIGN(cur_param[0]);
        else
            cur_param[0] = (y1-y2)/(x1-x2);
        cur_param[1] = y1-cur_param[0]*x1;
        if (abs(cur_param[0]) < SMALL_NUMBER) // flat line?
            cur_param[0] = SMALL_NUMBER * SIGN(cur_param[0]);


        // 3. determine inliers using the whole set
        for (int i=0; i<N; i++) {
            float y_fit = cur_param[0]*x[i] + cur_param[1];
            float x_fit = (y[i] - cur_param[1])/cur_param[0];
            if (max(abs(y[i]-y_fit), abs(x[i]-x_fit))  < maxError) { // block distance
                inliers[i] = true;
            } else {
                inliers[i] = false;
            }
        }

        // 4. re-calculate params via quadratic fit on all inliers
        RMSE = fitLine(x, y, N, cur_param, (const bool*)inliers);
        if (abs(cur_param[0]) < SMALL_NUMBER) // flat line?
            cur_param[0] = SMALL_NUMBER * SIGN(cur_param[0]);

        // 5. re-calculate inliers
        ninliers = 0;
        for (int i=0; i<N; i++) {
            float y_fit = cur_param[0]*x[i] + cur_param[1];
            float x_fit = (y[i] - cur_param[1])/cur_param[0];
            if (max(abs(y[i]-y_fit), abs(x[i]-x_fit))  < maxError) {
                inliers[i] = true;
                ninliers++;
            } else {
                inliers[i] = false;
            }
        }

        // 6. calculate the error
        RMSE = errorLine(x, y, N, cur_param, (const bool*)inliers);

        // found a better solution?
        if (best_ninliers < ninliers) {
            best_ninliers = ninliers;
            param[0] = cur_param[0];
            param[1] = cur_param[1];
            bestRMSE = RMSE;
        }

        // 7. check exit condition
        inliers_ratio = (float)best_ninliers/N;
        if (RMSE < RMSE_OK && inliers_ratio > INLIERS_RATIO_OK) {

            if (debug)
                cout<<"Breaking early after "<< iter+1<<" iterations"<<endl;

            break;
        }
    } // iterations

    if (debug)
        cout<<"inliers ratio = "<< inliers_ratio <<endl;

    // 8. recreate inliers for the best parameters
    for (int i=0; i<N; i++) {
        float y_fit = param[0]*x[i] + param[1];
        float x_fit = (y[i] - param[1])/param[0];
        if (max(abs(y[i]-y_fit), abs(x[i]-x_fit))   < maxError) { //  block distance
            inliers[i] = true;
        } else {
            inliers[i] = false;
        }
    }

    if (inlrs==NULL)
        delete inliers;

    if (debug)
        std::cout << "RMSE: " << bestRMSE << std::endl;
    return bestRMSE;
}

// generates the data give the random seed
void generateData(float* x, float* y, unsigned int seed, bool hasOutl) {

    // initialize pseudo-random generator
    srand (seed);

    for (int i=0; i<NDATA_UNIT_TEST; i++) {

        // uniform noise (negative and positive -50..50)
        float noise = (float)(rand() % 100-50) *
                NOISE_LEVEL_UNIT_TEST / 100.0f;
        //cout<<noise<<"; ";

        // shift
        int noutlisers = (float)NDATA_UNIT_TEST * OUTLIER_PROPORTION_UNIT_TEST;
        int start = NDATA_UNIT_TEST/2-noutlisers/2; // put outliers in the middle
        bool outlier = (i > start ) && (i < start + noutlisers);
        if (hasOutl && outlier)
            noise += SHIFT_OUTLIERS_UNIT_TEST ;

        x[i] = i-NDATA_UNIT_TEST/2; // center x data around 0
        y[i] = TRUE_PARAM_UNIT_TEST[0]*x[i]+TRUE_PARAM_UNIT_TEST[1]+noise;
        //cout<<"x, y = "<<x[i]<<"; "<<y[i]<<endl;

    }

}

// generates the data from pasted numbers
void generateDataPaste(float* x, float* y) {

    const int n = 63;

    // compiler will generate an error if n is inaccurate
    float data[n][2] = {
            {1, 1},
            {2, 1},
            {3, 1},
            {4, 1},
            {5, 1},
            {6, 1},
            {7, 1},
            {8, 1},
            {9, 1},
            {2, 2},
            {3, 2},
            {4, 2},
            {5, 2},
            {6, 2},
            {7, 2},
            {8, 2},
            {9, 2},
            {10, 2},
            {11, 2},
            {12, 2},
            {13, 2},
            {14, 2},
            {15, 2},
            {16, 2},
            {17, 2},
            {8, 3},
            {10, 3},
            {12, 3},
            {13, 3},
            {14, 3},
            {15, 3},
            {16, 3},
            {17, 3},
            {18, 3},
            {9, 4},
            {10, 4},
            {11, 4},
            {12, 4},
            {13, 4},
            {14, 4},
            {15, 4},
            {16, 4},
            {17, 4},
            {18, 4},
            {19, 4},
            {20, 4},
            {21, 4},
            {22, 4},
            {23, 4},
            {24, 4},
            {25, 4},
            {17, 5},
            {18, 5},
            {19, 5},
            {20, 5},
            {21, 5},
            {22, 5},
            {23, 5},
            {24, 5},
            {25, 5},
            {26, 5},
            {27, 5},
            {28, 5}};

    // crop or repeate data to get a required sample size
    for (int i=0; i<NDATA_UNIT_TEST; i++) {
        x[i] = data[i%n][0];
        y[i] = data[i%n][1];
    }

}

// unit test of quadratic line fit
bool uniTest_fitLine_QUADRATIC(int col, int row) {

    const bool hasOutliers = false; // cannot handle outliers well
    const bool dataFromParam = false; // either data from param or pasted ones

    string str = hasOutliers?" with outliers":" with no outliers";
    cout<<"unit-test_fitLine()"<<str<<endl;
    bool res = false;

    // opencv window
    Mat img(UNIT_TEST_IMG_HEIGHT, UNIT_TEST_IMG_WIDTH, CV_8U);
    cv::namedWindow("QUADRATIC", CV_WINDOW_AUTOSIZE);
    cv::moveWindow("QUADRATIC", col*(img.cols+50), row*(img.rows+50));

    // create data
    float x[NDATA_UNIT_TEST], y[NDATA_UNIT_TEST];
    if (dataFromParam)
        generateData(x, y, time(NULL), hasOutliers);
    else
        generateDataPaste(x, y);

    // fit the line
    float param[2];
    float er = fitLine(x, y, NDATA_UNIT_TEST, param) ;
    if (er < NOISE_LEVEL_UNIT_TEST || !dataFromParam)
        res = true;

    // print the result
    cout<<"a, b = "<<param[0]<<"; "<<param[1]<<"; RMSE = "<<er<<endl;

    if (dataFromParam) {
        cout<<"True a = "<<TRUE_PARAM_UNIT_TEST[0]<<"; b = "<<
                TRUE_PARAM_UNIT_TEST[1]<<endl;
        cout<<"combined param error = "<<
                abs(TRUE_PARAM_UNIT_TEST[0]-param[0])+
                abs(TRUE_PARAM_UNIT_TEST[1]-param[1])<<endl;
        if (res)
            cout<<"===passed"<<endl;
        else
            cout<<"===FAIL!"<<endl;
        cout<<endl;
    }
    // visualize
    showPoints(img, x, y, NDATA_UNIT_TEST, NULL, param);
    imshow("QUADRATIC", img);
    cv::waitKey(10);

    return res;
}

// unit test of RANSAC line fit
bool uniTest_fitLine_RANSAC(int col, int row) {

    const bool hasOutliers = true; // handles outliers gracefully
    const bool dataFromParam = false; // either data from param or pasted ones

    string str = hasOutliers?" with outliers":" with no outliers";
    cout<<"unit-test_RANSAC()"<<str<<endl;
    bool res = false;

    // opencv window
    Mat img(UNIT_TEST_IMG_HEIGHT, UNIT_TEST_IMG_WIDTH, CV_8U);
    cv::namedWindow("RANSAC", CV_WINDOW_AUTOSIZE);
    cv::moveWindow("RANSAC", col*(img.cols+50), row*(img.rows+50));

    // create data
    float x[NDATA_UNIT_TEST], y[NDATA_UNIT_TEST];
    if (dataFromParam)
        generateData(x, y, time(NULL), hasOutliers);
    else
        generateDataPaste(x, y);

    // parameters
    float param[2];
    int niter = 20;
    float maxError = 1.0f;
    bool inliers[NDATA_UNIT_TEST];
    bool debug = true;
    float er;

    // function call
    er = RANSAC_line(x, y, NDATA_UNIT_TEST, param, niter, maxError,
            inliers, debug);
    if (er < NOISE_LEVEL_UNIT_TEST || !dataFromParam)
        res = true;

    // print the result
    cout<<"outliers: ";
    int noutliers = 0;
    for (int i=0; i<NDATA_UNIT_TEST; i++) {
        if (!inliers[i]) {
            noutliers++;
            cout<<i<<"; ";
            if (noutliers % 30==0)
                cout<<endl;
        }
    }
    cout<<" overall "<<noutliers<<endl;
    cout<<"a, b = "<<param[0]<<"; "<<param[1]<<"; RMSE = "<<er<<endl;

    if (dataFromParam) {
        cout<<"True a = "<<TRUE_PARAM_UNIT_TEST[0]<<"; b = "<<
                TRUE_PARAM_UNIT_TEST[1]<<endl;
        cout<<"combined param error = "<<
                abs(TRUE_PARAM_UNIT_TEST[0]-param[0])+
                abs(TRUE_PARAM_UNIT_TEST[1]-param[1])<<endl;
        if (res)
            cout<<"===passed"<<endl;
        else
            cout<<"===FAIL!"<<endl;
        cout<<endl;
    }

    // visualize
    showPoints(img, x, y, NDATA_UNIT_TEST, inliers, param);
    imshow("RANSAC", img);
    cv::waitKey(10);

    return res;
}

void paramToVect(float* param, Point2f& normal, float* D) {

    normal.x = param[0];
    normal.y = -1;
    *D = -param[1];

    // length of normal
    float len = sqrt(normal.dot(normal)); // length of the normal
    assert (len > SMALL_NUMBER);
    float Z = 1.0/len;
    normal *= Z;
    *D *= Z;
}

void paramToPointVect(const float* param, cv::Vec4f &linePointVect)
{
    if (0.0f == param[0])
    {
        linePointVect[0] = 1.0f;        //vx
        linePointVect[1] = 0.0f;        //vy
        linePointVect[2] = 0.0f;        //x0
        linePointVect[3] = param[1];    //y0
    }
    else
    {
        float theta = std::atan(param[0]);
        float vx = std::cos(theta);
        float vy = std::sin(theta);

        float x0,y0;
        if (param[0] >= LARGE_NUMBER)
        {
            x0 = param[1];
            y0 = 0.0f;
        }
        else
        {
            x0 = 0.0f;
            y0 = param[1];
        }

        linePointVect[0] = vx;
        linePointVect[1] = vy;
        linePointVect[2] = x0;
        linePointVect[3] = y0;
    }
}
