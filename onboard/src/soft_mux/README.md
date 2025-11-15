# class `SoftMux` reference

## Inputs
### Topics
- `pwm_ctrl`: pwm from Matlab
- `pwm_cli`: pwm from the cli

### Services
- `Control_mode`: A service to set the mode. Return whether the mode is set sucessfully

## Outputs
### Topics
- `pwm_cmd`

## Pseudocode
```C++
class SoftMux : Node {
    public:
        // Constructor
        SoftMux();
        
        // Methods
        pwm_ctrl_callback(pwm) {
            if (is_matlab_mode) {
                pwm_cmd_publish(pwm);
            }
        }
        pwm_cli_callback(pwm) {
            if (!is_matlab_mode) {
                pwm_cmd_publish(pwm);
            }
        }

        set_mode_srv(const &request, &response) {
            // based on request, set the response            
            this->is_matlab_mode = request->mode;
            // if (this->is_matlab_mode == request->mode) {
            //     response->res = 1
            // } else {
            //     reponse->res = 0
            // }
            // // Log 
        }

    private:
        pwm_cmd_publish(pwm) {
            // set fields of pwm message to publish
            // publish
        }

        bool is_matlab_mode

        pwm_ctrl_subscriber
        pwm_cli_subscriber
        pwm_cmd_publisher
        control_mode_service
}
```