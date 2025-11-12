#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// 1. Includi il file header generato dalla tua action
#include "ros2_kdl_package/action/execute_trajectory.hpp"

// 2. !!! NUOVA DIPENDENZA !!!
// Poiché il tuo goal ora è un Point, devi includerlo
#include "geometry_msgs/msg/point.hpp"


// 3. Definisci i 'using' per leggibilità
using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

class TrajectoryActionClient : public rclcpp::Node
{
public:

double target_x_;
double target_y_;
double target_z_;

  explicit TrajectoryActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("execute_trajectory_client", options)
  {
    this->declare_parameter("end_position_x", 0.0);
    this->declare_parameter("end_position_y", 0.0);
    this->declare_parameter("end_position_z", 0.0);

    this->get_parameter("end_position_x", target_x_);
    this->get_parameter("end_position_y", target_y_);
    this->get_parameter("end_position_z", target_z_); 
    
    RCLCPP_INFO(this->get_logger(), "Goal caricato dal .yaml: [x: %.2f, y: %.2f, z: %.2f]",
        target_x_, target_y_, target_z_);

    // 4. Crea il client con lo stesso nome del server
    this->client_ptr_ = rclcpp_action::create_client<ExecuteTrajectory>(
      this, "execute_trajectory");

    RCLCPP_INFO(this->get_logger(), "Client creato. In attesa del server 'execute_trajectory'...");

    // 5. Attendi che il server sia disponibile
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Server Action non disponibile dopo 10 secondi.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Server Action trovato. Invio del goal.");
    this->send_goal();
  }

  void send_goal()
  {
    using namespace std::placeholders;

    // 6. Crea il messaggio di Goal
    auto goal_msg = ExecuteTrajectory::Goal();

    // !!! MODIFICA CHIAVE !!!
    // Ora impostiamo i campi .x, .y, .z del 'target_end_position'
    // come definito nel tuo file .action e usato dal tuo server.
    goal_msg.target_end_position.x = target_x_;
    goal_msg.target_end_position.y = target_y_;
    goal_msg.target_end_position.z = target_z_;

    RCLCPP_INFO(this->get_logger(), "Invio della posizione finale [x: %.2f, y: %.2f, z: %.2f]",
        goal_msg.target_end_position.x,
        goal_msg.target_end_position.y,
        goal_msg.target_end_position.z);

    // 7. Imposta le callback per gestire la risposta
    auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      std::bind(&TrajectoryActionClient::goal_response_callback, this, _1);
    
    send_goal_options.feedback_callback =
      std::bind(&TrajectoryActionClient::feedback_callback, this, _1, _2);
    
    send_goal_options.result_callback =
      std::bind(&TrajectoryActionClient::result_callback, this, _1);
    
    // 8. Invia il goal
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr client_ptr_;

  void goal_response_callback(const GoalHandleExecuteTrajectory::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal è stato RIFIUTATO dal server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal ACCETTATO dal server, in attesa del risultato...");
    }
  }

  void feedback_callback(
    GoalHandleExecuteTrajectory::SharedPtr,
    const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback)
  {
    // !!! MODIFICA CHIAVE !!!
    // Usiamo 'current_error_norm' come definito nel tuo file .action
    RCLCPP_INFO(this->get_logger(), "Feedback ricevuto: Errore di posizione = %.4f", feedback->current_error_norm);
  }

  void result_callback(const GoalHandleExecuteTrajectory::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        // !!! MODIFICA CHIAVE !!!
        // Usiamo 'success' come definito nel tuo file .action
        RCLCPP_INFO(this->get_logger(), "Goal RIUSCITO. Risultato: %s", result.result->success ? "true" : "false");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal ANNULLATO (Abort)");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal CANCELLATO");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Codice risultato sconosciuto");
        break;
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<TrajectoryActionClient>();
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}