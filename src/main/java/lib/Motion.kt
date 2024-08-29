package lib

import com.fasterxml.jackson.annotation.JsonInclude
import com.fasterxml.jackson.annotation.JsonProperty


@JsonInclude(JsonInclude.Include.NON_NULL)
data class MotionCommand(
    @JsonProperty("direction") val direction: String,
    @JsonProperty("value") val value: Double,
)

@JsonInclude(JsonInclude.Include.NON_NULL)
data class RobotMotion(
    @JsonProperty("commands") val commands: List<MotionCommand>
)