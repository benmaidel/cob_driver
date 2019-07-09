/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/package.h>

#include <cob_mimic/SetMimicAction.h>
#include <cob_mimic/SetMimicGoal.h>
#include <cob_mimic/SetMimicFeedback.h>
#include <cob_mimic/SetMimicResult.h>

#include <cob_mimic/SetMimic.h>
#include <cob_mimic/SetMimicRequest.h>
#include <cob_mimic/SetMimicResponse.h>

#include <vlc/vlc.h>
#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/chrono.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/uniform_int_distribution.hpp>

class Mimic
{
public:
    Mimic():
        real_dist_(2,10), int_dist_(0,6)
    {
    }

    ~Mimic(void)
    {
        libvlc_media_player_stop(vlc_player_);
        libvlc_media_player_release(vlc_player_);
        libvlc_release(vlc_inst_);
    }

    bool init()
    {
        if(!copy_mimic_files())
            return false;

        random_mimics_.push_back("blinking");
        random_mimics_.push_back("blinking");
        random_mimics_.push_back("blinking");
        random_mimics_.push_back("blinking_left");
        random_mimics_.push_back("blinking_right");

        int_dist_ = boost::random::uniform_int_distribution<>(0,static_cast<int>(random_mimics_.size())-1);

        char const *argv[] =
        {
            "--ignore-config",
            "--mouse-hide-timeout=0",
            "-q",
            "--no-osd",
            "-L",
            "--no-one-instance",
            "--playlist-enqueue",
            "--no-video-title-show",
            "--no-skip-frames",
            "--no-audio",
        };
        int argc = sizeof( argv ) / sizeof( *argv );

        vlc_inst_ = libvlc_new(argc, argv);
        if(!vlc_inst_){std::cerr<<"failed to create libvlc instance"<<std::endl; return false;}
        vlc_player_ = libvlc_media_player_new(vlc_inst_);
        if(!vlc_player_){std::cerr<<"failed to create vlc media player object"<<std::endl; return false;}

        libvlc_set_fullscreen(vlc_player_, 1);
        set_mimic("default", 1, 1.0, false);
        blinking_timer_ = boost::thread(&Mimic::blinking_cb, this);
        return true;
    }

    void blinking_cb()
    {
      while(true){
        int rand = int_dist_(gen_);
        set_mimic(random_mimics_[rand], 1, 1.5);
        //blinking_timer_ = nh_.createTimer(ros::Duration(real_dist_(gen_)), &Mimic::blinking_cb, this, true);
        boost::this_thread::sleep_for( boost::chrono::milliseconds( (int)(real_dist_(gen_))*1000) );
      }
    }

private:
    boost::thread blinking_timer_;
    std::string mimic_folder_;

    std::string active_mimic_;

    libvlc_instance_t* vlc_inst_;
    libvlc_media_player_t* vlc_player_;
    libvlc_media_t* vlc_media_;

    bool new_mimic_request_;
    boost::mutex mutex_;

    boost::random::mt19937 gen_;
    boost::random::uniform_real_distribution<> real_dist_;
    boost::random::uniform_int_distribution<> int_dist_;
    std::vector<std::string> random_mimics_;

    bool copy_mimic_files()
    {
        char *lgn;
        if((lgn = getlogin()) == NULL)
        {
            lgn = getenv("USER");
            if(lgn == NULL || std::string(lgn) == "")
            {
                std::cerr<<"unable to get user name"<<std::endl;
                return false;
            }
        }
        std::string username(lgn);
        mimic_folder_ = "/tmp/mimic_" + username;
        std::cout<<"copying all mimic files to "<< mimic_folder_.c_str()<<"..."<<std::endl;
        std::string pkg_path = ros::package::getPath("cob_mimic");
        std::string mimics_path = pkg_path + "/common";

        try{
            if(boost::filesystem::exists(mimic_folder_))
            {
                boost::filesystem::remove_all(mimic_folder_);
            }
        }
        catch(boost::filesystem::filesystem_error const & e)
        {
            std::cerr<<std::string(e.what())<<std::endl;
            return false;
        }

        if(copy_dir(boost::filesystem::path(mimics_path), boost::filesystem::path(mimic_folder_)) )
        {
            std::cout<<"...copied all mimic files to "<< mimic_folder_.c_str()<<std::endl;
            return true;
        }
        else
        {
            std::cout<<"...could not copy mimic files to "<< mimic_folder_.c_str()<<std::endl;
            return false;
        }
    }

    bool set_mimic(std::string mimic, int repeat, float speed, bool blocking=true)
    {
        new_mimic_request_=true;
        std::cout<<"New mimic request with: "<< mimic.c_str()<<std::endl;
        mutex_.lock();
        active_mimic_= (boost::format("Mimic: %1%, repeat: %2%, speed: %3%, blocking: %4%")% mimic % repeat % speed % blocking).str();
        new_mimic_request_=false;
        std::cout<<(boost::format("Mimic: %1% (speed: %2%, repeat: %3%)") % mimic % speed % repeat).str()<<std::endl;

        std::string filename = mimic_folder_ + "/" + mimic + ".mp4";

        // check if mimic exists
        if ( !boost::filesystem::exists(filename) )
        {
            if ( !boost::filesystem::exists(mimic) )
            {
                std::cerr<<"File not found: "<< filename.c_str()<<std::endl;
                active_mimic_ = "None";
                mutex_.unlock();
                return false;
            }
            else
            {
                std::cout<<"Playing mimic from non-default file: "<< mimic.c_str()<<std::endl;
                filename = mimic;
            }
        }

        // repeat cannot be 0
        repeat = std::max(1, repeat);

        // speed cannot be 0 or negative
        if(speed <= 0)
        {
            std::cout<<"Mimic speed cannot be 0 or negative. Setting Speed to 1.0"<<std::endl;
            speed = 1.0;
        }

        // returns -1 if an error was detected, 0 otherwise (but even then, it might not actually work depending on the underlying media protocol)
        if(libvlc_media_player_set_rate(vlc_player_, speed)!=0){std::cerr<<"failed to set movie play rate"<<std::endl;}

        while(repeat > 0)
        {
            vlc_media_ = libvlc_media_new_path(vlc_inst_, filename.c_str());
            if(!vlc_media_)
            {
                std::cerr<<"failed to create media for filepath "<< filename.c_str()<<std::endl;
                active_mimic_ = "None";
                mutex_.unlock();
                return false;
            }

            libvlc_media_player_set_media(vlc_player_, vlc_media_);
            libvlc_media_release(vlc_media_);

            // returns 0 if playback started (and was already started), or -1 on error.
            if(libvlc_media_player_play(vlc_player_)!=0)
            {
                std::cout<<"failed to play"<<std::endl;
                active_mimic_ = "None";
                mutex_.unlock();
                return false;
            }


            while(blocking && (libvlc_media_player_is_playing(vlc_player_) == 1))
            {

                std::cout<<"still playing "<< mimic.c_str()<<std::endl;
                if(new_mimic_request_)
                {
                    std::cout<<"mimic "<< mimic.c_str() << "preempted"<<std::endl;
                    active_mimic_ = "None";
                    mutex_.unlock();
                    return false;
                }
            }
            repeat --;
        }
        active_mimic_ = "None";
        mutex_.unlock();
        return true;
    }



    bool copy_dir( boost::filesystem::path const & source,
            boost::filesystem::path const & mimic_folder )
    {
        namespace fs = boost::filesystem;
        try
        {
            // Check whether the function call is valid
            if(!fs::exists(source) || !fs::is_directory(source))
            {
                std::cout<<"Source directory " << source.string() << " does not exist or is not a directory."<<std::endl;
                return false;
            }
            if(fs::exists(mimic_folder))
            {
                std::cerr<<"Destination directory " << mimic_folder.string() << " already exists."<<std::endl;
                return false;
            }
            // Create the mimic_folder directory
            if(!fs::create_directory(mimic_folder))
            {
                std::cerr<< "Unable to create mimic_folder directory" << mimic_folder.string()<<std::endl;
                return false;
            }
        }
        catch(fs::filesystem_error const & e)
        {
            std::cout<<std::string(e.what())<<std::endl;
            return false;
        }
        // Iterate through the source directory
        for(fs::directory_iterator file(source); file != fs::directory_iterator(); ++file)
        {
            try
            {
                fs::path current(file->path());
                if(fs::is_directory(current))
                {
                    // Found directory: Recursion
                    if( !copy_dir(current, mimic_folder / current.filename()) )
                        return false;
                }
                else
                {
                    // Found file: Copy
                    fs::copy_file(current, mimic_folder / current.filename() );
                }
            }
            catch(fs::filesystem_error const & e)
            {
                std::cout<<std::string(e.what())<<std::endl;
            }
        }
        return true;
    }
};


int main(int argc, char** argv)
{
    Mimic mimic;
    if(!mimic.init())
    {
        std::cerr<<"mimic init failed"<<std::endl;
        return 1;
    }
    else
    {
        boost::thread t1 = boost::thread(&Mimic::blinking_cb, &mimic);
        boost::thread t2 = boost::thread(&Mimic::blinking_cb, &mimic);
        boost::thread t3 = boost::thread(&Mimic::blinking_cb, &mimic);
        t1.join();
        t2.join();
        t3.join();
    }
}
