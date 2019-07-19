#include "opencv2/opencv.hpp"
//#include "opencv2/core/core.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <iostream>
#include <vector>
#include <queue>
#include <stdlib.h>


using namespace cv;
using namespace std;

struct group{
	Point bottom;
	Point top;
	Point left;
	Point right;
	float angle;
	int length = 0;  
};

struct containers{
	vector<group> groups;
	int minx;
	int maxx;
	float angle;
};

int threshold = 2550;
int angle_tolerance = 180;
int allowed_perp_dist = 40;
vector<group> groups;

Mat colour_final_detected_lane_cluters(Mat coloured,Mat image,Point start,int r,int g,int b,int horizon){
	queue<Point> v;
	v.push(start);
	int e=0;
	while(v.size())
	{
		//cout << "hehe" <<endl;
		Point current = v.front();
		
		v.pop();
		coloured.at<Vec3b>(current.x,current.y) = {b,g,r};
		
		//cout << "ok" << endl;
		int limit = 12;
		if(abs(current.x - horizon) < 10){
			limit = 2;
		}
		else if(abs(current.x - horizon) < 20){
			limit = 3;
		}
		else if(abs(current.x - horizon) < 50){
			limit = 6;
		}
		for(int k=current.x+limit;k>=current.x-limit;k--){
			for(int l=current.y-limit;l<=current.y+limit;l++){
				if(!(k>=0 && l>=0 && k<image.rows && l<image.cols)) continue;
				//cout <<"abe"<<endl;
				if(!(coloured.at<Vec3b>(k,l)[0] == b && coloured.at<Vec3b>(k,l)[1] == g && coloured.at<Vec3b>(k,l)[2] == r) && image.at<uchar>(k,l)>127){
					//cout << k << " " <<l <<endl;
					
					coloured.at<Vec3b>(k,l) = {b,g,r};
					e++;
					
					//imshow("Original",marked);
					//cout << "b" <<endl;
					if(k<=current.x) v.push(Point(k,l));
				} 
 			}
		}
		//cout << v.size() <<endl;
	}
	//cout << e <<endl;
	return coloured;
}

void arrange(){
	int i,j;
	for(i=0;i<groups.size()-1;i++){
		group min = groups[i];int minx=i;
		for(j=i+1;j<groups.size();j++){
			if(groups[j].bottom.x>min.bottom.x){
				minx = j;min = groups[j];
			}
			
		}
		groups[i] = groups[minx];
		groups[minx] = min;

	}
}

Mat markgroup(Mat image,Mat marked, int i,int j,int color,int horizon){
	queue<Point> v;
	v.push(Point(i,j));
	int e=0;
	while(v.size())
	{
		//cout << "hehe" <<endl;
		Point current = v.front();
		
		v.pop();
		marked.at<uchar>(current.x,current.y) = color;
		
		//cout << "ok" << endl;
		int limit = 12;
		if(abs(current.x - horizon) < 10){
			limit = 2;
		}
		else if(abs(current.x - horizon) < 20){
			limit = 3;
		}
		else if(abs(current.x - horizon) < 50){
			limit = 6;
		}
		for(int k=current.x+limit;k>=current.x-limit;k--){
			for(int l=current.y-limit;l<=current.y+limit;l++){
				if(!(k>=0 && l>=0 && k<image.rows && l<image.cols)) continue;
				//cout <<"abe"<<endl;
				if(!marked.at<uchar>(k,l)&& image.at<uchar>(k,l)>127){
					//cout << k << " " <<l <<endl;
					
					marked.at<uchar>(k,l) = color;
					e++;
					if(k<groups[groups.size()-1].top.x) groups[groups.size()-1].top = Point(k,l);
					if(k>groups[groups.size()-1].bottom.x) groups[groups.size()-1].bottom = Point(k,l);
					if(l<groups[groups.size()-1].left.y) groups[groups.size()-1].left = Point(k,l);
					if(l>groups[groups.size()-1].right.y) groups[groups.size()-1].right = Point(k,l);
					groups[groups.size()-1].length++;

					//imshow("Original",marked);
					//cout << "b" <<endl;
					if(k<=current.x) v.push(Point(k,l));
				} 
 			}
		}
		//cout << v.size() <<endl;
	}
	//cout << e <<endl;
	return marked;
}

Mat markgroups(Mat image,int horizon){
	Mat marked = image.clone();int noofsegments = 0;
	for(int i = 0; i < image.rows ; i++) for(int j=0;j<image.cols;j++) marked.at<uchar>(i,j) = 0;
	int color = 60;
	for(int i=image.rows-1;i>=0;i--){
		//cout << i <<endl; 
		for(int j=0;j<image.cols;j++){
			if(image.at<uchar>(i,j)>127 && !marked.at<uchar>(i,j)) {
				cout << color <<endl;
				group g;
				groups.push_back(g);
				groups[groups.size()-1].left = groups[groups.size()-1].right = groups[groups.size()-1].top = groups[groups.size()-1].bottom = Point(i,j);
				
				marked = markgroup(image,marked,i,j,color,horizon);
				//if(groups[groups.size()-1].length<20) groups.remove(groups.size());
				marked.at<uchar>(groups[groups.size()-1].left.x,groups[groups.size()-1].left.y) = 255;
				marked.at<uchar>(groups[groups.size()-1].right.x,groups[groups.size()-1].right.y) = 255;
				marked.at<uchar>(groups[groups.size()-1].top.x,groups[groups.size()-1].top.y) = 255;
				marked.at<uchar>(groups[groups.size()-1].bottom.x,groups[groups.size()-1].bottom.y) = 255;
				color+=60;
				color%=253;
			}
		}
	}
	
	//cout << "color : " <<color<<endl;
	
	return marked;
}

int Gethorizon(Mat image){
	int i=0;
	while(true){
		int no = 0;
		for(int j=0;j<image.cols;j++){
			no+= image.at<uchar>(i,j);
		}
		if(no>2550) break;
		i++;
	}
	for(int j=0;j<image.cols;j++){
		image.at<uchar>(i,j)=127 ;
	}
	return i;
}

int main(int argv,char** argc) {			
	Mat imge = imread(argc[1],1);
	Mat image;
	cvtColor(imge, image, CV_BGR2GRAY);
	for(int i=0;i<imge.rows;i++){
		for(int j=0;j<imge.cols;j++){
			imge.at<Vec3b>(i,j) = {0,0,0};
		}
	}
	
	
	int horizon;
	cout << "Started" <<endl;
	Mat img2 = image.clone();
	//erode(image,img2,Mat(),Point(-1,-1),1,BORDER_CONSTANT, morphologyDefaultBorderValue());
	//dilate(img2,image,Mat(),Point(-1,-1),1,BORDER_CONSTANT, morphologyDefaultBorderValue());
	Mat img = image.clone();
	horizon = Gethorizon(image);
	namedWindow("Original",WINDOW_NORMAL);
	namedWindow("Segmented",WINDOW_NORMAL);
	image = markgroups(image,horizon);
	imshow("Segmented",image);
	imshow("Original",img);
	arrange();
	int i,j;int k;int no=0;
	containers container[4];
	for(int i=0;i<4;i++) container[i].maxx = 0;
	for(i=0;i<groups.size();i++){
		if(groups[i].length<20) continue;
		groups[i].angle = (groups[i].right.x-groups[i].left.x+groups[i].bottom.y-groups[i].top.y>0) ? -atan((float)(groups[i].bottom.x-groups[i].top.x)/(groups[i].right.y-groups[i].left.y))*(180/3.14) + 180 : atan((float)(groups[i].bottom.x-groups[i].top.x)/(groups[i].right.y-groups[i].left.y))*(180/3.14);
		cout << "Group with angle : " << groups[i].angle <<endl;  
		int maxdist=0;int maxdistpos;bool check1 = false;bool check2 = false;
		for(j=0;j<4;j++){
			if(groups[i].bottom.x>container[j].maxx){
				cout << "Vapas bhej diya " << j << " " << container[j].angle<<endl;
				continue;
			}
			cout << "Angle difference  : " << abs(groups[i].angle - container[j].angle) <<endl;
			if(abs(groups[i].angle - container[j].angle) < angle_tolerance){
				float slope = tan(container[j].angle*3.14/180);
				Point mid_point = Point((groups[i].top.x+groups[i].bottom.x)/2,(groups[i].left.y+groups[i].right.y)/2);
				cout << mid_point.x << "," << mid_point.y << "'" <<slope<<endl;
				float distance = abs((mid_point.x + slope*(mid_point.y-container[j].groups[container[j].groups.size()-1].top.y) - container[j].groups[container[j].groups.size()-1].top.x)/(sqrt(1+slope*slope)));
				cout << distance <<endl;
				if(distance < allowed_perp_dist && (distance < maxdist || maxdist==0)){
					maxdist = distance;
					maxdistpos = j;
					check1=true;
				}
				
			}
			//else if(!check1){ 
				//float slope1 = tan(container[j].angle);
			//}
		}
		cout <<"yaha aa gaya"<< check1 << " " << check2 << " " << no << endl;
		if(check1){
			container[maxdistpos].groups.push_back(groups[i]);
			container[maxdistpos].maxx = groups[i].top.x;
			container[maxdistpos].angle = (container[maxdistpos].groups[0].bottom.y-groups[i].top.y>0) ? -atan((float)(container[maxdistpos].groups[0].bottom.x-groups[i].top.x)/(container[maxdistpos].groups[0].bottom.y-groups[i].top.y))*(180/3.14) + 180 : atan((float)(container[maxdistpos].groups[0].bottom.x-groups[i].top.x)/(groups[i].top.y-container[maxdistpos].groups[0].bottom.y))*(180/3.14);
			
			continue;
		}
		else if(check2){
			container[maxdistpos].groups.push_back(groups[i]);
			container[maxdistpos].maxx = groups[i].top.x;
			container[maxdistpos].angle = (container[maxdistpos].groups[0].bottom.y-groups[i].top.y>0) ? -atan((float)(container[maxdistpos].groups[0].bottom.x-groups[i].top.x)/(container[maxdistpos].groups[0].bottom.y-groups[i].top.y))*(180/3.14) + 180 : atan((float)(container[maxdistpos].groups[0].bottom.x-groups[i].top.x)/(groups[i].top.y-container[maxdistpos].groups[0].bottom.y))*(180/3.14);
			continue;
		}	
		if(no<4) {
			cout << "yes,addition into new container " << no <<endl;
			container[no].groups.push_back(groups[i]);
			container[no].maxx = groups[i].top.x;
			container[no].minx = groups[i].bottom.x;
			container[no++].angle = groups[i].angle;
		}
	}
	cout << no <<endl;int r,g,b;
	for(int i =0;i<no;i++){
		cout << "blue" <<endl;
		if(i==0){
			r=255;b=0;g=0;
		}
		if(i==1){
			r=0;b=255;g=0;
		}
		if(i==2){
			r=0;b=0;g=255;
		}
		if(i==3){
			r=255;b=255;g=0;
		}
		cout << container[i].groups.size() <<endl;
		for(int j=0;j<container[i].groups.size();j++){
			cout << " red" <<endl;
			imge = colour_final_detected_lane_cluters(imge,img,Point(container[i].groups[j].bottom.x,container[i].groups[j].bottom.y),b,g,r,horizon);
		}
	}
	namedWindow("Final");
	imshow("Final",imge);
	waitKey(0);
	return 0;
}