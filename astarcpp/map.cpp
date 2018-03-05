#include <iostream>
//#include <random>
#include <cstdlib>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


float fade(float t){
	return t*t*t*(t*(t*6-15)+10);
}
float lerp(float a,float b,float t){
	return a + t * (b - a);
	//return (1.0-t)*a + t*b;
}
float dot(float* a, float* b, int n){
	float res=0;
	for(int i=0; i<n; ++i){
		res += a[i]*b[i];
	}
	return res;
}

class Stats{
	float mn, mx;
	public:
	Stats(){
		mn = FLT_MAX;
		mx = -FLT_MAX;
	}
	void operator()(float x){
		mn = mn<x?mn:x;
		mx = mx>x?mx:x;
	}
	void report(){
		std::cout << mn << ',' << mx << std::endl;
	}
};


template<int L=8, int N=(1<<L)>
class PerlinMapGenerator{
	private:
		float _decay;
		float _map[N][N];
		float _grad[N+1][N+1][2];
	public:
		PerlinMapGenerator(float decay=0.6)
			:_decay(decay){
				for(int i=0; i<L; ++i){
					_build(i);
				}
			}
		void _reset(){
			std::memset((float*)_map, 0, sizeof(_map));

		}
		void _g1(float x[2]){
			//float a = 2*M_PI * (rand() / float(RAND_MAX));
			//x[0] = cos(a);
			//x[1] = sin(a);

			static float g_src[4][2] ={
				{0, 1},
				{1, 0},
				{0, -1},
				{-1, 0}
			};

			int idx = (rand() % 4);
			x[0] = g_src[idx][0];
			x[1] = g_src[idx][1];
			//x[2] = (rand()%2)?-1:1;
		}
		void _build(int level){
			srand(time(0));
			if(level == 0){
				_reset();
			}
			float a = pow(_decay, level);

			// # cells per grid
			int res = (1 << (L - level));

			int m = 1 << level;
			// create gradient grid ...
			for(int i=0; i<N+1; ++i){
				for(int j=0; j<N+1; ++j){
					_g1(_grad[i][j]);
				}
			}

			//std::cout << N << ',' << level << ',' << m << ',' << res << std::endl;
			//Stats slx, srx, sty, sby;

			// create
			for(int i=0; i<N; ++i){
				for(int j=0; j<N; ++j){

					int t = i/res;
					int b = t+1;
					int l = j/res;
					int r = l+1;

					float y = i + 0.5;
					float x = j + 0.5;

					float lx = (x - l*res) / float(res);
					float rx = lx-1;//-1+lx;

					float ty = (y - t*res) / float(res);
					float by = ty-1;//-1+ty;

					//std::cout << lx << std::endl;
					// influence ...
					float i_tl = _grad[t][l][0]*lx + _grad[t][l][1]*ty;
					float i_tr = _grad[t][r][0]*rx + _grad[t][r][1]*ty;
					float i_bl = _grad[b][l][0]*lx + _grad[b][l][1]*by;
					float i_br = _grad[b][r][0]*rx + _grad[b][r][1]*by;

					float u = fade(lx);
					float v = fade(ty);

					float x1 = lerp(i_tl, i_tr, u);
					float x2 = lerp(i_bl, i_br, u);
					_map[i][j] += a * lerp(x1,x2,v);
				}
			}

		}

		float* data() {
			return (float*) _map;
		}
};

class CVRenderer{
	std::string name;
	public:
	CVRenderer(std::string name):name(name){
		open();
	}
	~CVRenderer(){
		//close();
	}
	void open(){
		cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
	}
	void show(const cv::Mat& mat){
		/*
		 * wait < 0 : no wait
		 * wait = 0 : stop
		 * wait > 0 : delay
		 */
		cv::imshow(name, mat);
	}
	void close(){
		cv::destroyWindow(name);
	}
};

void color_region(
		cv::Mat& m, cv::Mat& h, cv::Mat& s,
		float low, float high,
		float r, float g, float b
		){
	cv::inRange(h, cv::Scalar(low), cv::Scalar(high), s);
	m.setTo(cv::Scalar(b,g,r), s);
}

#define DIM 9
int main(){
	PerlinMapGenerator<9> gen;

	// data --> cv format
	auto m = cv::Mat(1<<DIM, 1<<DIM, CV_32FC1, gen.data());
	double mn, mx;
	cv::minMaxLoc(m,&mn,&mx);
	//std::cout << mn << ',' << mx << std::endl;

	// normalize ...
	m -= mn;
	m /= (mx-mn);

	cv::Mat m_col;
	cv::cvtColor(m, m_col, cv::COLOR_GRAY2BGR);
	cv::Mat sel;

	//color_region(m_col, m, sel, 0, 0.1, 0, 0, 0.502);
	//color_region(m_col, m, sel, 0.1, 0.2, 0, 0, 0.804);
	//color_region(m_col, m, sel, 0.2, 0.4, 0.118, 0.565, 1.0);
	//color_region(m_col, m, sel, 0.4, 0.5, 0.0, 0.749, 1.0);
	//color_region(m_col, m, sel, 0.5, 0.55, 1, 0.973, 0.863);
	//color_region(m_col, m, sel, 0.55, 0.7, 0.18, 0.545, 0.341);
	//color_region(m_col, m, sel, 0.7, 0.8, 0.0, 0.392, 0.0);
	//color_region(m_col, m, sel, 0.8, 0.9, 0.627, 0.322, 0.176);
	//color_region(m_col, m, sel, 0.9, 1.0, 0.941, 0.973, 1.0);

	color_region(m_col, m, sel, 0, 0.4, 0, 0, 0); // black
	color_region(m_col, m, sel, 0.4, 0.6, 1, 1, 1); // white
	color_region(m_col, m, sel, 0.6, 1.0, 0, 0, 0); // black

	cv::Mat m_gray;
	cv::cvtColor(m_col, m_gray, cv::COLOR_BGR2GRAY);
	cv::Mat m_grayu8;
	m_gray.convertTo(m_grayu8, CV_8UC1);

	// compute "navigable" path
	
	int dfactor = 3;	
	cv::Mat ker = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*dfactor+1,2*dfactor+1), cv::Point(dfactor,dfactor));
	cv::Mat m_nav;
	cv::erode(m_gray, m_nav, ker, cv::Point(-1,-1));

	cv::Mat m_navu8;
	m_nav.convertTo(m_navu8, CV_8UC1);

	cv::Mat m_lab;
	cv::Mat m_stats, m_cent;
	int n_labels = cv::connectedComponentsWithStats(m_navu8, m_lab, m_stats, m_cent, 8);

	std::vector<cv::Vec3b> colors(n_labels);
	colors[0] = cv::Vec3b(0,0,0);
	for(int i=1; i<n_labels;++i){
		colors[i] = cv::Vec3b(rand()&255, rand()&255, rand()&255);
	}

	cv::Mat m_cxn(m_lab.size(), CV_8UC3);
	for(int i=0; i<m_cxn.rows; ++i){
		for(int j=0; j<m_cxn.cols; ++j){
			int l = m_lab.at<int>(i,j);
			cv::Vec3b& pxl = m_cxn.at<cv::Vec3b>(i,j);
			pxl = colors[l];
		}
	}

	// find biggest region ...
	int mxi = 1;
	int mxa = 0;
	for(int i=1; i<n_labels; ++i){
		int a = m_stats.at<int>(i, cv::CC_STAT_AREA);
		if(a>mxa){
			mxa=a;
			mxi=i;
		}
	}
	std::cout << mxi << std::endl;

	// choose starting point ...
	int h = m_lab.rows;
	int w = m_lab.cols;
	int srci, srcj, dsti, dstj;

	while(true){
		int i = rand() % h;
		int j = rand() % w;
		if(m_lab.at<int>(i,j) == mxi){
			srci = i;
			srcj = j;
			break;
		}
	}

	while(true){
		int i = rand() % h;
		int j = rand() % w;
		if(m_lab.at<int>(i,j) == mxi){
			dsti = i;
			dstj = j;
			break;
		}
	}

	cv::circle(m_cxn, cv::Point(srcj, srci), 10, cv::Scalar(255,0,0), -1);
	cv::circle(m_cxn, cv::Point(dstj, dsti), 10, cv::Scalar(0,255,255), -1);

	//// to path
	//cv::RNG rng(12345);

	//cv::Mat m_grayu8;
	//m_gray.convertTo(m_grayu8, CV_8UC1);

	//cv::Mat m_edge;
	//std::vector<std::vector<cv::Point>> contours;
	//std::vector<cv::Vec4i> hierarchy;

	//std::cout << m.size << std::endl;

	////cv::blur(m_grayu8, m_grayu8, cv::Size(3,3));
	//cv::Mat ker = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3), cv::Point(1,1));

	//cv::Canny(m_grayu8, m_edge, 0.5, 1.0, 3);
	//cv::dilate(m_edge, m_edge, ker, cv::Point(-1,-1), 2);
	//cv::erode(m_edge, m_edge, ker, cv::Point(-1,-1), 2);

	//cv::findContours(m_edge, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));


	//cv::Mat m_cnt = cv::Mat::zeros(m_edge.size(), CV_8UC3);

	//for( int i = 0; i< int(contours.size()); i++ )
	//{
	//	cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	//	drawContours(m_cnt, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
	//}

	CVRenderer ren("window");
	CVRenderer ren2("window2");
	//CVRenderer ren3("window3");

	ren.show(m_nav);
	ren2.show(m_cxn);
	//ren3.show(m_gray);
	

	bool quit = false;
	while(!quit){
		switch(cv::waitKey(0)){
			case 27:
				quit = true;
				break;
		}
	}
}
