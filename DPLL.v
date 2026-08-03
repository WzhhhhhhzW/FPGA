`timescale 1ns / 1ps

// Reusable digital phase-alignment loop for offset-binary ADC samples.
// Closed loop: feedback_enable=1, feedback_data must be the DA analog output
// sampled by another ADC channel. phase_select_90 selects either 0 or 90 degrees.
module DigitalPLL #(
    parameter integer DATA_WIDTH = 10,
    parameter integer ADDR_WIDTH = 16
) (
    input  wire                  clk,
    input  wire [DATA_WIDTH-1:0] reference_data,
    input  wire [DATA_WIDTH-1:0] feedback_data,
    input  wire                  feedback_enable,
    input  wire                  phase_hold,
    input  wire                  phase_select_90,
    input  wire                  tight_lock,
    input  wire                  coarse_search,
    output wire [DATA_WIDTH-1:0] data_out,
    output wire                  locked,
    output reg                   frequency_locked,
    output reg                   phase_locked,
    output reg  [ADDR_WIDTH-1:0] period_samples
);

localparam [DATA_WIDTH-1:0] MID_SCALE = (1 << (DATA_WIDTH-1));
localparam [DATA_WIDTH-1:0] HYSTERESIS = 4;

(* ram_style = "block" *) reg [DATA_WIDTH-1:0]
    sample_history [0:(1 << ADDR_WIDTH)-1];

reg [ADDR_WIDTH-1:0] write_addr;
reg [ADDR_WIDTH-1:0] read_addr;
reg [DATA_WIDTH-1:0] delayed_sample;
reg [ADDR_WIDTH-1:0] timestamp;
reg [ADDR_WIDTH-1:0] last_reference_time;
reg [ADDR_WIDTH-1:0] reference_count;
reg reference_armed;
reg feedback_armed;
reg first_reference_seen;
reg previous_feedback_enable;
reg previous_phase_select_90;
reg previous_tight_lock;
reg [2:0] phase_lock_confirm_count;
reg [ADDR_WIDTH-1:0] applied_delay_samples;
localparam [1:0] PHASE_CALC_IDLE  = 2'd0;
localparam [1:0] PHASE_CALC_ERROR = 2'd1;
localparam [1:0] PHASE_CALC_APPLY = 2'd2;
reg [1:0] phase_calc_state;
reg [ADDR_WIDTH-1:0] phase_position_snapshot;
reg [ADDR_WIDTH-1:0] phase_period_snapshot;
reg [ADDR_WIDTH-1:0] phase_target_snapshot;
reg [ADDR_WIDTH-1:0] phase_delay_snapshot;
reg [ADDR_WIDTH-1:0] phase_error_magnitude_reg;
reg phase_adjust_add_reg;
reg [DATA_WIDTH-1:0] reference_center;
reg [DATA_WIDTH-1:0] feedback_center;
reg [DATA_WIDTH-1:0] reference_min;
reg [DATA_WIDTH-1:0] reference_max;
reg [DATA_WIDTH-1:0] feedback_min;
reg [DATA_WIDTH-1:0] feedback_max;

wire [DATA_WIDTH:0] reference_high_threshold =
    {1'b0, reference_center} + HYSTERESIS;
wire [DATA_WIDTH:0] feedback_high_threshold =
    {1'b0, feedback_center} + HYSTERESIS;
wire [DATA_WIDTH-1:0] reference_low_threshold =
    (reference_center > HYSTERESIS) ?
    (reference_center - HYSTERESIS) : {DATA_WIDTH{1'b0}};
wire [DATA_WIDTH-1:0] feedback_low_threshold =
    (feedback_center > HYSTERESIS) ?
    (feedback_center - HYSTERESIS) : {DATA_WIDTH{1'b0}};

wire reference_crossing =
    reference_armed &&
    ({1'b0, reference_data} >= reference_high_threshold);
wire feedback_crossing =
    feedback_armed &&
    ({1'b0, feedback_data} >= feedback_high_threshold);

// 0 degrees requires no period offset; 90 degrees is exactly one quarter
// period.  The loop adjusts the delay directly in samples, so no multiplier
// or divider is required in the timing-critical path.
wire [ADDR_WIDTH-1:0] selected_delay = applied_delay_samples;
wire [ADDR_WIDTH-1:0] target_position =
    phase_select_90 ? (period_samples >> 2) :
                      {ADDR_WIDTH{1'b0}};

// Position of the feedback positive crossing inside the current input period.
wire [ADDR_WIDTH-1:0] feedback_age =
    timestamp - last_reference_time;
wire [ADDR_WIDTH-1:0] feedback_position =
    (feedback_age >= period_samples) ?
    (feedback_age - period_samples) : feedback_age;

// Pipelined phase calculation.  Stage 1 works only from snapshots captured
// on a feedback crossing, breaking the former timestamp-to-delay carry chain.
wire [ADDR_WIDTH-1:0] phase_forward_error_stage =
    (phase_target_snapshot >= phase_position_snapshot) ?
    (phase_target_snapshot - phase_position_snapshot) :
    (phase_period_snapshot -
     (phase_position_snapshot - phase_target_snapshot));
wire [ADDR_WIDTH-1:0] phase_reverse_error_stage =
    (phase_forward_error_stage == {ADDR_WIDTH{1'b0}}) ?
    {ADDR_WIDTH{1'b0}} :
    (phase_period_snapshot - phase_forward_error_stage);
wire [ADDR_WIDTH-1:0] phase_error_magnitude_stage =
    (phase_forward_error_stage <= phase_reverse_error_stage) ?
    phase_forward_error_stage : phase_reverse_error_stage;

// Stage 2 uses the registered shortest error to select a coarse or fine step.
// Mode 3 must acquire quickly at the low end of its 1 kHz--8 kHz range.
// Waiting one reference period between corrections made the former 1/8-error
// step visibly slow at 1 kHz.  Tight lock uses the complete measured error;
// the other coarse-search mode keeps the gentler legacy correction.
wire [ADDR_WIDTH-1:0] coarse_phase_step =
    tight_lock ? phase_error_magnitude_reg :
                 (phase_error_magnitude_reg >> 3);
wire [ADDR_WIDTH-1:0] phase_adjust_step =
    coarse_search &&
    (coarse_phase_step != {ADDR_WIDTH{1'b0}}) ?
    coarse_phase_step : {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
wire [ADDR_WIDTH:0] delay_plus_step =
    {1'b0, phase_delay_snapshot} + {1'b0, phase_adjust_step};

assign locked = feedback_enable ? phase_locked : frequency_locked;
assign data_out =
    (applied_delay_samples == {ADDR_WIDTH{1'b0}}) ? reference_data :
    (frequency_locked ? delayed_sample : MID_SCALE);

initial begin
    write_addr               = {ADDR_WIDTH{1'b0}};
    read_addr                = {ADDR_WIDTH{1'b0}};
    delayed_sample           = MID_SCALE;
    timestamp                = {ADDR_WIDTH{1'b0}};
    last_reference_time      = {ADDR_WIDTH{1'b0}};
    reference_count          = {ADDR_WIDTH{1'b0}};
    reference_armed          = 1'b0;
    feedback_armed           = 1'b0;
    first_reference_seen     = 1'b0;
    previous_feedback_enable = 1'b0;
    previous_phase_select_90 = 1'b0;
    previous_tight_lock      = 1'b0;
    phase_lock_confirm_count = 3'd0;
    applied_delay_samples    = {ADDR_WIDTH{1'b0}};
    phase_calc_state         = PHASE_CALC_IDLE;
    phase_position_snapshot  = {ADDR_WIDTH{1'b0}};
    phase_period_snapshot    = {ADDR_WIDTH{1'b0}};
    phase_target_snapshot    = {ADDR_WIDTH{1'b0}};
    phase_delay_snapshot     = {ADDR_WIDTH{1'b0}};
    phase_error_magnitude_reg = {ADDR_WIDTH{1'b0}};
    phase_adjust_add_reg     = 1'b0;
    reference_center         = MID_SCALE;
    feedback_center          = MID_SCALE;
    reference_min            = {DATA_WIDTH{1'b1}};
    reference_max            = {DATA_WIDTH{1'b0}};
    feedback_min             = {DATA_WIDTH{1'b1}};
    feedback_max             = {DATA_WIDTH{1'b0}};
    frequency_locked         = 1'b0;
    phase_locked             = 1'b0;
    period_samples           = {ADDR_WIDTH{1'b0}};
end

always @(posedge clk) begin
    timestamp <= timestamp + 1'b1;

    sample_history[write_addr] <= reference_data;
    write_addr <= write_addr + 1'b1;
    read_addr <= write_addr - selected_delay;
    delayed_sample <= sample_history[read_addr];

    // Track the true center code of each waveform.  Separate adaptive centers
    // prevent ADC/DAC DC offsets from appearing as a false phase error.
    if (reference_data < reference_min)
        reference_min <= reference_data;
    if (reference_data > reference_max)
        reference_max <= reference_data;
    if (feedback_data < feedback_min)
        feedback_min <= feedback_data;
    if (feedback_data > feedback_max)
        feedback_max <= feedback_data;

    // Hysteretic positive-crossing detectors reject noise around each center.
    if (reference_data <= reference_low_threshold)
        reference_armed <= 1'b1;
    else if (reference_crossing)
        reference_armed <= 1'b0;

    if (feedback_data <= feedback_low_threshold)
        feedback_armed <= 1'b1;
    else if (feedback_crossing)
        feedback_armed <= 1'b0;

    // Reference frequency/period tracking.
    if (reference_crossing) begin
        last_reference_time <= timestamp;
        if (first_reference_seen) begin
            period_samples <= reference_count;
            frequency_locked <= 1'b1;
            reference_center <=
                ({1'b0, reference_min} + {1'b0, reference_max}) >> 1;
            feedback_center <=
                ({1'b0, feedback_min} + {1'b0, feedback_max}) >> 1;
        end
        reference_min <= reference_data;
        reference_max <= reference_data;
        feedback_min <= feedback_data;
        feedback_max <= feedback_data;
        first_reference_seen <= 1'b1;
        reference_count <= {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
    end
    else if (!(&reference_count)) begin
        reference_count <= reference_count + 1'b1;
    end
    else begin
        frequency_locked <= 1'b0;
        phase_locked <= 1'b0;
        phase_lock_confirm_count <= 3'd0;
        phase_calc_state <= PHASE_CALC_IDLE;
    end

    previous_feedback_enable <= feedback_enable;
    previous_phase_select_90 <= phase_select_90;
    previous_tight_lock <= tight_lock;

    if ((phase_select_90 != previous_phase_select_90) ||
        (tight_lock != previous_tight_lock)) begin
        // A new requested phase/accuracy must be reacquired before lock.
        phase_locked <= 1'b0;
        phase_lock_confirm_count <= 3'd0;
        applied_delay_samples <= target_position;
        phase_calc_state <= PHASE_CALC_IDLE;
    end
    else if (phase_hold) begin
        // Freeze the acquired correction while a downstream block changes
        // the output frequency and the feedback is no longer comparable.
        applied_delay_samples <= applied_delay_samples;
        phase_locked <= phase_locked;
        phase_calc_state <= PHASE_CALC_IDLE;
    end
    else if (!feedback_enable) begin
        applied_delay_samples <= target_position;
        phase_locked <= 1'b0;
        phase_lock_confirm_count <= 3'd0;
        phase_calc_state <= PHASE_CALC_IDLE;
    end
    else if (!previous_feedback_enable) begin
        // Start closed-loop acquisition from the selected nominal phase.
        applied_delay_samples <= target_position;
        phase_locked <= 1'b0;
        phase_lock_confirm_count <= 3'd0;
        phase_calc_state <= PHASE_CALC_IDLE;
    end
    else begin
        case (phase_calc_state)
            PHASE_CALC_IDLE: begin
                if (feedback_crossing && frequency_locked) begin
                    phase_position_snapshot <= feedback_position;
                    phase_period_snapshot <= period_samples;
                    phase_target_snapshot <= target_position;
                    phase_delay_snapshot <= applied_delay_samples;
                    phase_calc_state <= PHASE_CALC_ERROR;
                end
            end

            PHASE_CALC_ERROR: begin
                phase_error_magnitude_reg <=
                    phase_error_magnitude_stage;
                phase_adjust_add_reg <=
                    (phase_forward_error_stage <
                     (phase_period_snapshot >> 1));
                phase_calc_state <= PHASE_CALC_APPLY;
            end

            PHASE_CALC_APPLY: begin
                if (phase_error_magnitude_reg <=
                    {{(ADDR_WIDTH-1){1'b0}}, 1'b1}) begin
                    if (tight_lock) begin
                        // Two consecutive in-band crossings reject a
                        // transient while avoiding four extra low-frequency
                        // periods before mode 3 switches to the doubler.
                        if (phase_lock_confirm_count >= 3'd1) begin
                            phase_lock_confirm_count <=
                                phase_lock_confirm_count;
                            phase_locked <= 1'b1;
                        end
                        else begin
                            phase_lock_confirm_count <=
                                phase_lock_confirm_count + 1'b1;
                            phase_locked <= 1'b0;
                        end
                    end
                    else begin
                        phase_lock_confirm_count <= 3'd0;
                        phase_locked <= 1'b1;
                    end
                end
                else if (phase_adjust_add_reg) begin
                    // More delay advances the measured phase around the
                    // circular period.
                    phase_locked <= 1'b0;
                    phase_lock_confirm_count <= 3'd0;
                    if (delay_plus_step >=
                        {1'b0, phase_period_snapshot})
                        applied_delay_samples <=
                            delay_plus_step -
                            {1'b0, phase_period_snapshot};
                    else
                        applied_delay_samples <=
                            delay_plus_step[ADDR_WIDTH-1:0];
                end
                else begin
                    phase_locked <= 1'b0;
                    phase_lock_confirm_count <= 3'd0;
                    if (phase_delay_snapshot >= phase_adjust_step)
                        applied_delay_samples <=
                            phase_delay_snapshot - phase_adjust_step;
                    else
                        applied_delay_samples <=
                            phase_period_snapshot -
                            (phase_adjust_step - phase_delay_snapshot);
                end
                phase_calc_state <= PHASE_CALC_IDLE;
            end

            default: begin
                phase_calc_state <= PHASE_CALC_IDLE;
            end
        endcase
    end
end

endmodule
