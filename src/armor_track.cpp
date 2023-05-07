#include "armor_track.h"
#include "gimbal_control.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

//#define DRAW_MATCH_ARMOR
#define SHOW_TRACK_PREDICT
#define SHOW_SINGER_PREDICT

//namespace robot_detection {

ArmorTracker::ArmorTracker()
{
    cv::FileStorage fs("../other/track_data.yaml", cv::FileStorage::READ);

    KF.initial_KF();

    locate_target = false;
    enemy_armor = Armor();

    tracker_state = MISSING;
    tracking_id = 0;

    find_aim_cnt = 0;
    find_threshold = (int)fs["find_threshold"];

    lost_aim_cnt = 0;
    lost_threshold = (int)fs["lost_threshold"];

    new_old_threshold = (double)fs["new_old_threshold"];

    isChangeSameID = false;
    fs.release();
}

void ArmorTracker::reset()
{
    t = -1;

    KF.initial_KF();
    Singer.Reset();

    locate_target = false;
    enemy_armor = Armor();
    tracker_state = MISSING;
    tracking_id = 0;
    find_aim_cnt = 0;
    lost_aim_cnt = 0;
}

// 初始化，选择最优装甲板，设置卡尔曼的F和x_1
bool ArmorTracker::initial(std::vector<Armor> &find_armors)
{
    if(find_armors.empty())
    {
        return false;
    }

    sort(find_armors.begin(),find_armors.end(),
      [](Armor &armor1,Armor &armor2){
         return armor1.grade > armor2.grade;});

    // select enemy
    enemy_armor = find_armors[0];
    tracker_state = DETECTING;
    tracking_id = enemy_armor.id;

    KF.initial_KF();
    enemy_armor.imu_position = AS.pixel2imu(enemy_armor);
    KF.setXPost(enemy_armor.imu_position);
    Singer.setXpos(enemy_armor.imu_position.head(2));
//    std::cout<<"track_initial"<<std::endl;
    return true;
}

// dt是两帧之间时间间隔, 跟得住目标
bool ArmorTracker::selectEnemy2(std::vector<Armor> &find_armors, double dt)
{

    KF.setF(dt);
    predicted_enemy = KF.predict();
#ifdef SHOW_TRACK_PREDICT
//    cv::Mat PointTrack = AS._src.clone();
    Eigen::Vector3d predictedE =  Eigen::Vector3d(predicted_enemy.head(3));
    cv::Point2f preTrack = AS.imu2pixel(predictedE);
    cv::circle(AS._src,preTrack,5,cv::Scalar(0,0,255),-1);
    cv::imshow("_src",AS._src);
#endif

    Armor matched_armor;
    bool matched = false;

    if(!find_armors.empty())
    {
        double min_position_diff = DBL_MAX;
        for(auto & armor : find_armors)
        {
            armor.imu_position= AS.pixel2imu(armor);
            Eigen::Vector3d pre = predicted_enemy.head(3);
            double position_diff = (pre - armor.imu_position).norm();
            if (position_diff < min_position_diff) {
                min_position_diff = position_diff;
                matched_armor = armor;
            }
        }
//        std::cout<<"min_position_diff:"<<min_position_diff<<std::endl;
        if (min_position_diff < new_old_threshold && matched_armor.id == tracking_id) {
            matched = true;
            predicted_enemy = KF.update(matched_armor.imu_position);
        }
        else
        {
            // 本帧内是否有相同ID
            for (auto & armor : find_armors) {
                if (armor.id == tracking_id) {
                    matched = true;
//                    armor.imu_position = AS.pixel2imu(armor);
                    KF.initial_KF();
                    Eigen::Matrix<double,6,1> pos_speed;
                    pos_speed << armor.imu_position, predicted_enemy.tail(3);
                    KF.setPosAndSpeed(armor.imu_position,predicted_enemy.tail(3));
                    predicted_enemy = pos_speed;
                    matched_armor = armor;
                    break;
                }
            }
        }
    }
    if (matched)
    {
#ifdef DRAW_MATCH_ARMOR
        cv::Mat m_a = AS._src.clone();
        cv::circle(m_a,matched_armor.center,matched_armor.size.width/10,cv::Scalar(255,255,0),-1);
        Eigen::Vector3d predicted_track;
        predicted_track << predicted_enemy.head(3);
        cv::Point2f p = AS.imu2pixel(predicted_track);
        cv::circle(m_a,p,matched_armor.size.width/15,cv::Scalar(0,0,255),-1);
        cv::imshow("DRAW_MATCH_ARMOR",m_a);
#endif
        enemy_armor = matched_armor;
    }

    if (tracker_state == DETECTING) {
        // DETECTING
        if (matched) {
            find_aim_cnt++;
            if (find_aim_cnt > find_threshold) {
                find_aim_cnt = 0;
                tracker_state = TRACKING;
            }
        } else {
            find_aim_cnt = 0;
            tracker_state = MISSING;
        }

    } else if (tracker_state == TRACKING) {
        // TRACKING
        if (!matched) {
            tracker_state = LOSING;
            lost_aim_cnt++;
        }

    } else if (tracker_state == LOSING) {
        if (!matched) {
            lost_aim_cnt++;
            if (lost_aim_cnt > lost_threshold) {
                lost_aim_cnt = 0;
                tracker_state = MISSING;
            }
        } else {
            tracker_state = TRACKING;
            lost_aim_cnt = 0;
        }
    }

    if (tracker_state == MISSING)
    {
        reset();
        return false;
    }
    KF.setPosAndSpeed(matched_armor.imu_position,predicted_enemy.tail(3));

    if(tracker_state == LOSING)
    {
        enemy_armor.imu_position = predicted_enemy.head(3);
        KF.setPosAndSpeed(enemy_armor.imu_position,predicted_enemy.tail(3));
    }

    return true;
}

// 对处于跟踪和正在丢失状态时做 预测，引入各种时间
bool ArmorTracker::estimateEnemy(double dt)
{
    cv::putText(AS._src, std::to_string(tracker_state),cv::Point(50,100),1,5,cv::Scalar(0,0,255),3);
    if(tracker_state == TRACKING || tracker_state == LOSING)
    {
        // enemy_armor get real information to predicted by singer
        enemy_armor.imu_position = AS.pixel2imu(enemy_armor);
        std::cout<<"[target_distance]:  "<<enemy_armor.imu_position(0,0)<<std::endl;

        double fly_time = AS.getFlyTime(enemy_armor.imu_position);
        ////////////////Singer predictor//////////////////////////////
        if(!Singer.SingerPrediction(dt,fly_time,
                                    enemy_armor.imu_position,
                                    predicted_position))
        {
            return false;
        }
		////////////////Singer predictor//////////////////////////////
        bullet_point = AS.airResistanceSolve(predicted_position);
#ifdef SHOW_SINGER_PREDICT
        cv::Point2f pixelPos = AS.imu2pixel(bullet_point);
//        std::cout<<"pixelPos"<<pixelPos<<std::endl;
		circle(AS._src,enemy_armor.center,5,cv::Scalar(255,0,0),-1);
		circle(AS._src,pixelPos,5,cv::Scalar(0,255,255),-1);
		for (int i=0;i<4;i++)
		{
		    line(AS._src, enemy_armor.armor_pt4[i], enemy_armor.armor_pt4[(i + 1) % 4], CV_RGB(0, 255, 255),2,cv::LINE_8);
		}
        cv::imshow("_src",AS._src);
        //        if(tracker_state == LOSING)cv::waitKey(0);
#endif
		return true;
    }
    else if (tracker_state == DETECTING)
    {
        enemy_armor.imu_position = AS.pixel2imu(enemy_armor);
        std::cout << "[target_distance]:  "<<enemy_armor.imu_position(0,0)<<std::endl;
        double fly_time = AS.getFlyTime(enemy_armor.imu_position);
        ////////////////Singer predictor//////////////////////////////
        if(!Singer.SingerPrediction(dt,fly_time,
                                    enemy_armor.imu_position,
                                    predicted_position))
        {
            return false;
        }
        ////////////////Singer predictor//////////////////////////////
#ifdef SHOW_SINGER_PREDICT
        cv::Point2f pixelPos = AS.imu2pixel(bullet_point);
//        std::cout<<"pixelPos"<<pixelPos<<std::endl;
		circle(AS._src,enemy_armor.center,5,cv::Scalar(255,0,0),-1);
        circle(AS._src,pixelPos,5,cv::Scalar(0,0,255),-1);
        for (int i=0;i<4;i++)
        {
            line(AS._src, enemy_armor.armor_pt4[i], enemy_armor.armor_pt4[(i + 1) % 4], CV_RGB(0, 255, 255),2,cv::LINE_8);
        }
        cv::imshow("_src",AS._src);
//        cv::waitKey(0);
#endif
        locate_target = false;
        return false;
    }
    else
    {
        locate_target = false;
        return false;
    }
}

bool ArmorTracker::locateEnemy(const cv::Mat& src, std::vector<Armor> &armors, double time)
{
    //!< 先搞清楚电控那边的坐标系是怎么样的(准确的陀螺仪本身定义的xyz在哪个方向，不能是人为定义的)，
    //!< 特别是现在他们统一了c板的安装（佛山的时候理清过，现在忘了）
    //!< 理清电控那边imu的坐标系后，拿到的陀螺仪的四元数才是可用的，
    //!< 确定imu坐标系xyz（！！！！一定是陀螺仪自身定义的，不是人为定义的），
    //!< imu坐标系中心平移到云台旋转轴中心！！！（佛山的时候缺失的一步，大概率也是导致当时测试失败的一步，而且一定是在四元数作用前平移
    //!< ，准确来说应该是相机坐标系的中心，因为上面说的imu和相机只是三轴方向不同，中心是一样的）
    //!< 拿到四元数用来转旋转矩阵作用到云台转轴中心的坐标系，得到我们diaski的世界坐标系的点
	AS._src = src;
	_src=src;
//	std::cout<<"AS.ab_roll:"<<AS.ab_roll<<std::endl;
//	std::cout<<"AS.ab_yaw:"<<AS.ab_yaw<<std::endl;
//	std::cout<<"AS.ab_pitch:"<<AS.ab_pitch<<std::endl;
//	std::cout<<"AS.bullet_speed:"<<AS.bullet_speed<<std::endl;
	Eigen::Vector3d euler_imu = {(double)AS.ab_roll/180.0*CV_PI,(double)AS.ab_pitch/180.0*CV_PI,(double)AS.ab_yaw/180.0*CV_PI};  // xyz-rpy
	AS.RotationMatrix_imu = AS.eulerAnglesToRotationMatrix2(euler_imu);
    if(!locate_target)
    {
        if(initial(armors))
        {
            locate_target = true;
        }
        else
        {
            locate_target = false;
        }
        Eigen::Vector3d gun_offset = {0,0,0};   // m
        gun_offset +=enemy_armor.camera_position;

        bullet_point = AS.airResistanceSolve(gun_offset);

        pitch = -atan2(bullet_point[1],bullet_point[2])/CV_PI*180 + AS.ab_pitch;
        yaw   = -atan2(bullet_point[0],bullet_point[2])/CV_PI*180 + AS.ab_yaw;
        return false;
    }
    else
    {
        if (t == -1)
        {
            t = time;
            return false;
        }
        double dt = (time - t) / (double)cv::getTickFrequency();
//        std::cout<<"dt:"<<dt<<std::endl;
        t = time;
        if(!selectEnemy2(armors,dt))
        {
            return false;
        }

        if(!estimateEnemy(dt))
        {
            return false;
        }

        Eigen::Vector3d gun_offset = {0,0,0};   // m
        gun_offset +=enemy_armor.camera_position;
        bullet_point = AS.airResistanceSolve(gun_offset);

        return true;
    }
}

void ArmorTracker::show()
{
    cv::putText(_src,"PITCH    : "+std::to_string(pitch),cv::Point2f(0,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(_src,"YAW      : "+std::to_string(yaw),cv::Point2f(0,90),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(_src,"DISTANCE : "+std::to_string(enemy_armor.camera_position.norm())+"m",cv::Point2f(0,30),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::Point2f vertice_lights[4];
    enemy_armor.points(vertice_lights);
    for (int i = 0; i < 4; i++) {
        line(_src, vertice_lights[i], vertice_lights[(i + 1) % 4], CV_RGB(0, 255, 255),2,cv::LINE_8);
    }
    std::string information = std::to_string(enemy_armor.id) + ":" + std::to_string(enemy_armor.confidence*100) + "%";
    //        putText(final_armors_src,ff,finalArmors[i].center,FONT_HERSHEY_COMPLEX, 1.0, Scalar(12, 23, 200), 1, 8);
    putText(_src, information,enemy_armor.armor_pt4[3],cv::FONT_HERSHEY_COMPLEX,2,cv::Scalar(255,0,255),1,3);

    imshow("final_result",_src);

}

//}
