#include "drake/multibody/rigid_body_plant/drake_visualizer.h"

#include <chrono>
#include <thread>

#include "drake/common/text_logging.h"
#include "drake/systems/rendering/drake_visualizer_client.h"
#include "drake/lcmt_viewer2_comms.hpp"

namespace drake {
namespace systems {

namespace {
// Defines the index of the port that the DrakeVisualizer uses.
const int kPortIndex = 0;
}  // namespace

DrakeVisualizer::DrakeVisualizer(const RigidBodyTree<double>& tree,
                                 drake::lcm::DrakeLcmInterface* lcm,
                                 bool enable_playback)
    : lcm_(lcm),
      load_message_(CreateLoadMessage(tree)),
      draw_message_translator_(tree),
      tree_(&tree) {
  set_name("drake_visualizer");
  const int vector_size =
      tree.get_num_positions() + tree.get_num_velocities();
  DeclareInputPort(kVectorValued, vector_size);
  if (enable_playback) log_.reset(new SignalLog<double>(vector_size));
}

void DrakeVisualizer::set_publish_period(double period) {
  LeafSystem<double>::DeclarePublishPeriodSec(period);
}

void DrakeVisualizer::ReplayCachedSimulation() const {
  if (log_ != nullptr) {
    // Build piecewise polynomial
    auto times = log_->sample_times();
    // NOTE: The SignalLog can record signal for multiple identical time stamps.
    //  This culls the duplicates as required by the PiecewisePolynomial.
    std::vector<int> included_times;
    included_times.reserve(times.rows());
    std::vector<double> breaks;
    included_times.push_back(0);
    breaks.push_back(times(0));
    int last = 0;
    for (int i = 1; i < times.rows(); ++i) {
      double val = times(i);
      if (val != breaks[last]) {
        breaks.push_back(val);
        included_times.push_back(i);
        ++last;
      }
    }

    auto sample_data = log_->data();
    std::vector<MatrixX<double>> knots;
    knots.reserve(sample_data.cols());
    for (int c : included_times) {
      knots.push_back(sample_data.col(c));
    }
    auto func = PiecewisePolynomial<double>::ZeroOrderHold(breaks, knots);

    PlaybackTrajectory(func);
  } else {
    drake::log()->warn(
        "DrakeVisualizer::ReplayCachedSimulation() called on instance that "
        "wasn't initialized to record. Next time, please construct "
        "DrakeVisualizer with recording enabled.");
  }
}

void PublishLine(const std::string json_str, drake::lcm::DrakeLcmInterface* lcm, long long int now) {
  auto msg = lcmt_viewer2_comms();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto& c : json_str) {
    msg.data.push_back(c);
    // std::cout << c;
  }
  // std::cout << std::endl << std::endl << std::endl;
  msg.num_bytes = json_str.size();
  // Use channel 0 for remote viewer communications.
  // std::cout << json_str << std::endl;

  const int lcm_message_length = msg.getEncodedSize();
  std::vector<uint8_t> lcm_message_bytes{};
  lcm_message_bytes.resize(lcm_message_length);
  msg.encode(lcm_message_bytes.data(), 0, lcm_message_length);

  lcm->Publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", lcm_message_bytes.data(),
      lcm_message_length);
}

void DrawCables(const RigidBodyTree<double>* tree,
                  drake::lcm::DrakeLcmInterface* lcm,
                  const BasicVector<double>& state, long long int now) {
  // TODO(geronm) use state + tree to make lines positions. For now, it will be random
  std::string json_1 = "{\"delete\":{},\"setgeometry\":[{\"geometry\":{\"color\":[0.0,0.0,0.0,1.0],\"points\":";
  std::string json_2 = ",\"type\":\"line\"},\"path\":[\"";
  std::string json_3 = "\"]}],\"settransform\":{},\"timestamp\":";
  std::string json_4 = "}";

  // Make the points in a vector:
  if (tree->constraint_cables.size() > 0) {
    Eigen::VectorXd q(tree->get_num_positions());
    for (int i=0; i<tree->get_num_positions(); ++i) {
      q(i) = state.GetAtIndex(i);
    }
    KinematicsCache<double> cache = tree->doKinematics(q);
    for (size_t c = 0; c < tree->constraint_cables.size(); ++c) {
      std::string points_str="";
      {
        auto wrapPoints = tree->constraint_cables[c].getWrapPoints(cache);

        std::ostringstream stringStream;
        stringStream << "[";
        for (int row = 0; row < wrapPoints.rows(); row++) {
          if (row > 0) {
            stringStream << ",";
          }
          stringStream << "[";
          for (int col = 0; col < wrapPoints.cols(); col++) {
            if (col > 0) {
              stringStream << ",";
            }
            stringStream << wrapPoints(row, col);
          }
          stringStream << "]";
        }
        stringStream << "]";

        points_str = stringStream.str();
      }

      // std::string dummy_points = "[[1.86657557780192,-0.464247709762942,0.747660221127523],[-1.53370505391172,1.3916316561968,0.504459205256841],[-1.73012210768299,1.39219064159683,-0.427621228005646],[2.73162407272025,-0.31940360668027,0.381326330271418]]";
      std::string path_prefix = "test_line";
      // std::string dummy_time = "1492025357657361";
     
      std::ostringstream stringStream;
      stringStream << json_1 << points_str << json_2 << path_prefix << c << json_3 << now << json_4;
      std::string json_str = stringStream.str();

      PublishLine(json_str, lcm, now);
    }
  }
}

void DrakeVisualizer::PlaybackTrajectory(
    const PiecewisePolynomial<double>& input_trajectory) const {
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  using TimePoint = std::chrono::time_point<Clock, Duration>;

  // Target frame length at 60 Hz playback rate.
  const double kFrameLength = 1 / 60.0;
  double sim_time = input_trajectory.getStartTime();
  TimePoint prev_time = Clock::now();
  BasicVector<double> data(log_->get_input_size());
  while (sim_time < input_trajectory.getEndTime()) {
    data.set_value(input_trajectory.value(sim_time));

    // Translates the input vector into an array of bytes representing an LCM
    // message.
    std::vector<uint8_t> message_bytes;
    draw_message_translator_.Serialize(sim_time, data,
                                       &message_bytes);

    // Publishes onto the specified LCM channel.
    lcm_->Publish("DRAKE_VIEWER_DRAW", message_bytes.data(),
                  message_bytes.size());

    // Also draw cables.
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto value = now_ms.time_since_epoch();
    long long int duration = value.count();
    DrawCables(this->tree_, lcm_, data, duration);

    const TimePoint earliest_next_frame = prev_time + Duration(kFrameLength);
    std::this_thread::sleep_until(earliest_next_frame);
    TimePoint curr_time = Clock::now();
    sim_time += (curr_time - prev_time).count();
    prev_time = curr_time;
  }

  // Final evaluation is at the final time stamp, guaranteeing the final state
  // is visualized.
  data.set_value(input_trajectory.value(input_trajectory.getEndTime()));
  std::vector<uint8_t> message_bytes;
  draw_message_translator_.Serialize(sim_time, data,
                                     &message_bytes);
  lcm_->Publish("DRAKE_VIEWER_DRAW", message_bytes.data(),
                message_bytes.size());
}

void DrakeVisualizer::DoPublish(const Context<double>& context) const {
  // TODO(liang.fok): Replace the following code once System 2.0's API allows
  // systems to declare that they need a certain action to be performed at
  // simulation time t_0.
  //
  // Before any draw commands, we need to send the load_robot message.
  if (context.get_time() == 0.0) {
    PublishLoadRobot();
  }
  DRAKE_DEMAND(sent_load_robot_);

  // Obtains the input vector, which contains the generalized q,v state of the
  // RigidBodyTree.
  const BasicVector<double>* input_vector = EvalVectorInput(context,
                                                            kPortIndex);
  if (log_ != nullptr) {
    log_->AddData(context.get_time(), input_vector->get_value());
  }

  // Translates the input vector into an array of bytes representing an LCM
  // message.
  std::vector<uint8_t> message_bytes;
  draw_message_translator_.Serialize(context.get_time(), *input_vector,
                                     &message_bytes);

  // Also draw cables.
  auto now = std::chrono::system_clock::now();
  auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
  auto value = now_ms.time_since_epoch();
  long long int duration = value.count();
  // BasicVector<double> data(log_->get_input_size());
  // BasicVector<double> input_vector_local(input_vector->size());
  // input_vector_local.SetFromVector(input_vector->CopyToVector());
  DrawCables(this->tree_, lcm_, *input_vector, duration);

  // Publishes onto the specified LCM channel.
  lcm_->Publish("DRAKE_VIEWER_DRAW", message_bytes.data(),
                message_bytes.size());
}

void DrakeVisualizer::PublishLoadRobot() const {
  const int lcm_message_length = load_message_.getEncodedSize();
  std::vector<uint8_t> lcm_message_bytes{};
  lcm_message_bytes.resize(lcm_message_length);
  load_message_.encode(lcm_message_bytes.data(), 0, lcm_message_length);

  lcm_->Publish("DRAKE_VIEWER_LOAD_ROBOT", lcm_message_bytes.data(),
      lcm_message_length);
  sent_load_robot_ = true;
}

lcmt_viewer_load_robot DrakeVisualizer::CreateLoadMessage(
    const RigidBodyTree<double>& tree) {
  lcmt_viewer_load_robot load_message;
  load_message.num_links = tree.bodies.size();
  for (const auto& body : tree.bodies) {
    lcmt_viewer_link_data link;
    link.name = body->get_name();
    link.robot_num = body->get_model_instance_id();
    link.num_geom = body->get_visual_elements().size();
    for (const auto& visual_element : body->get_visual_elements()) {
      link.geom.push_back(rendering::MakeGeometryData(visual_element));
    }
    load_message.link.push_back(link);
  }

  return load_message;
}

}  // namespace systems
}  // namespace drake
