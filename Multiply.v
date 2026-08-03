`timescale 1ns / 1ps

// Ping-pong buffered frequency doubler.
// One bank captures the current complete AD1 period while the other bank
// replays the previous complete period at twice the captured time rate.
module Multiply #(
    parameter integer DATA_WIDTH = 10,
    parameter integer ADDR_WIDTH = 16
) (
    input  wire                  clk,
    input  wire                  enable,
    input  wire [DATA_WIDTH-1:0] data_in,
    input  wire [DATA_WIDTH-1:0] feedback_data,
    input  wire [ADDR_WIDTH-1:0] period_samples,
    input  wire                  period_locked,
    output reg  [DATA_WIDTH-1:0] data_out,
    output reg                   phase_locked,
    output wire [ADDR_WIDTH-1:0] debug_completed_length,
    output wire [ADDR_WIDTH-1:0] debug_write_address,
    output wire signed [ADDR_WIDTH:0] debug_phase_trim,
    output wire signed [ADDR_WIDTH:0] debug_phase_trim_pending,
    output wire [ADDR_WIDTH-1:0] debug_folded_feedback_position
);

localparam [DATA_WIDTH-1:0] MID_SCALE = (1 << (DATA_WIDTH-1));
localparam [DATA_WIDTH-1:0] HYSTERESIS = 4;
localparam integer FILTER_SHIFT = 3;
localparam integer RAM_ADDR_WIDTH = 15;
localparam integer BANK_ADDR_WIDTH = RAM_ADDR_WIDTH - 1;
localparam [ADDR_WIDTH-1:0] BANK_DEPTH = (1 << BANK_ADDR_WIDTH);

wire [RAM_ADDR_WIDTH-1:0] ram_write_address;
wire [RAM_ADDR_WIDTH-1:0] ram_read_address;
wire [DATA_WIDTH-1:0] ram_read_data;
wire ram_write_enable;

reg write_bank;
reg read_bank;
reg capture_active;
reg completed_valid;
reg [ADDR_WIDTH-1:0] write_address;
reg [ADDR_WIDTH-1:0] completed_length;
reg [ADDR_WIDTH-1:0] playback_address;
reg [ADDR_WIDTH:0] playback_phase_accum;
reg signed [ADDR_WIDTH:0] phase_trim;
reg signed [ADDR_WIDTH:0] phase_trim_pending;
reg trim_update_waiting;
reg [2:0] phase_lock_confirm_count;
reg [1:0] phase_unlock_count;
reg [1:0] downsample_count;

// Independent adaptive centers prevent DC offsets from becoming phase error.
reg input_armed;
reg feedback_armed;
reg [DATA_WIDTH-1:0] input_center;
reg [DATA_WIDTH-1:0] input_min;
reg [DATA_WIDTH-1:0] input_max;
reg [DATA_WIDTH-1:0] feedback_center;
reg [DATA_WIDTH-1:0] feedback_min;
reg [DATA_WIDTH-1:0] feedback_max;
reg [DATA_WIDTH+FILTER_SHIFT-1:0] input_filter_acc;
reg [DATA_WIDTH+FILTER_SHIFT-1:0] feedback_filter_acc;

wire [DATA_WIDTH-1:0] input_filtered =
    input_filter_acc[DATA_WIDTH+FILTER_SHIFT-1:FILTER_SHIFT];
wire [DATA_WIDTH-1:0] feedback_filtered =
    feedback_filter_acc[DATA_WIDTH+FILTER_SHIFT-1:FILTER_SHIFT];

wire [DATA_WIDTH:0] input_high_threshold =
    {1'b0, input_center} + HYSTERESIS;
wire [DATA_WIDTH-1:0] input_low_threshold =
    (input_center > HYSTERESIS) ?
    (input_center - HYSTERESIS) : {DATA_WIDTH{1'b0}};
wire [DATA_WIDTH:0] feedback_high_threshold =
    {1'b0, feedback_center} + HYSTERESIS;
wire [DATA_WIDTH-1:0] feedback_low_threshold =
    (feedback_center > HYSTERESIS) ?
    (feedback_center - HYSTERESIS) : {DATA_WIDTH{1'b0}};

wire input_crossing =
    input_armed && ({1'b0, input_filtered} >= input_high_threshold);
wire feedback_crossing =
    feedback_armed &&
    ({1'b0, feedback_filtered} >= feedback_high_threshold);

// A complete input period is reduced by four before being stored.  The MSB
// selects the ping-pong bank and the lower 14 bits address that bank.
wire regular_capture_sample =
    capture_active && (downsample_count == 2'd3) &&
    (write_address < BANK_DEPTH);
assign ram_write_enable =
    enable && period_locked && (input_crossing || regular_capture_sample);
assign ram_write_address =
    input_crossing ?
        {(capture_active ? ~write_bank : write_bank),
         {BANK_ADDR_WIDTH{1'b0}}} :
        {write_bank, write_address[BANK_ADDR_WIDTH-1:0]};

// Adding the signed trim changes phase without modifying either stable bank.
wire signed [ADDR_WIDTH:0] trimmed_address_signed =
    $signed({1'b0, playback_address}) + phase_trim;
reg [ADDR_WIDTH-1:0] trimmed_read_address;
always @* begin
    if (completed_length == {ADDR_WIDTH{1'b0}})
        trimmed_read_address = {ADDR_WIDTH{1'b0}};
    else if (trimmed_address_signed < 0)
        trimmed_read_address =
            trimmed_address_signed + $signed({1'b0, completed_length});
    else if (trimmed_address_signed >=
             $signed({1'b0, completed_length}))
        trimmed_read_address =
            trimmed_address_signed - $signed({1'b0, completed_length});
    else
        trimmed_read_address = trimmed_address_signed[ADDR_WIDTH-1:0];
end

// Calculate the first address of a newly completed bank with the same active
// phase correction.  This avoids an address-0 sample followed immediately by
// a jump to phase_trim on the next clock.
reg [BANK_ADDR_WIDTH-1:0] bank_switch_start_address;
always @* begin
    if (write_address == {ADDR_WIDTH{1'b0}})
        bank_switch_start_address = {BANK_ADDR_WIDTH{1'b0}};
    else if (phase_trim < 0)
        bank_switch_start_address =
            $signed({1'b0, write_address}) + phase_trim;
    else if (phase_trim >= $signed({1'b0, write_address}))
        bank_switch_start_address =
            phase_trim - $signed({1'b0, write_address});
    else
        bank_switch_start_address =
            phase_trim[BANK_ADDR_WIDTH-1:0];
end

// Harmonic feedback phase detector.  AD2 has two positive crossings during
// each AD1 period, so its position is folded into a half-period interval.
reg [ADDR_WIDTH-1:0] reference_age;
wire [ADDR_WIDTH-1:0] half_period = period_samples >> 1;
wire [ADDR_WIDTH-1:0] folded_feedback_position =
    input_crossing ? {ADDR_WIDTH{1'b0}} :
    ((reference_age >= half_period) ?
     (reference_age - half_period) : reference_age);
wire [ADDR_WIDTH-1:0] second_crossing_window_start =
    half_period >> 1;
wire [ADDR_WIDTH-1:0] second_crossing_window_end =
    half_period + (half_period >> 1);
wire selected_feedback_crossing =
    feedback_crossing &&
    (reference_age >= second_crossing_window_start) &&
    (reference_age < second_crossing_window_end);

// Read-only debug taps for the top-level ILA. These assignments do not
// participate in the multiplier or DA2 data path.
assign debug_completed_length = completed_length;
assign debug_write_address = write_address;
assign debug_phase_trim = phase_trim;
assign debug_phase_trim_pending = phase_trim_pending;
assign debug_folded_feedback_position = folded_feedback_position;

// Use the feedback crossing around the middle of the input period.  It is
// separated from the ping-pong bank switch and avoids correcting twice from
// the two nominally equivalent crossings of the doubled waveform.
wire [ADDR_WIDTH-1:0] folded_reverse_error =
    (folded_feedback_position == {ADDR_WIDTH{1'b0}}) ?
    {ADDR_WIDTH{1'b0}} :
    (half_period - folded_feedback_position);
wire [ADDR_WIDTH-1:0] harmonic_phase_error =
    (folded_feedback_position <= folded_reverse_error) ?
    folded_feedback_position : folded_reverse_error;
wire phase_inside_acquire_band =
    harmonic_phase_error <= {{(ADDR_WIDTH-2){1'b0}}, 2'd2};
wire phase_inside_release_band =
    harmonic_phase_error <= {{(ADDR_WIDTH-3){1'b0}}, 3'd6};
wire trim_increment_direction =
    folded_feedback_position < (half_period >> 1);
// Only one stored address is requested at a time.  A new request is blocked
// until the old request has been committed at a playback boundary.  This
// prevents delayed feedback updates from accumulating into a visible jump.
wire signed [ADDR_WIDTH:0] trim_one_signed =
    $signed({{ADDR_WIDTH{1'b0}}, 1'b1});
wire signed [ADDR_WIDTH:0] trim_upper_limit =
    $signed({1'b0, completed_length}) - trim_one_signed;
wire signed [ADDR_WIDTH:0] trim_lower_limit =
    -$signed({1'b0, completed_length}) + trim_one_signed;
wire signed [ADDR_WIDTH:0] trim_incremented =
    phase_trim + trim_one_signed;
wire signed [ADDR_WIDTH:0] trim_decremented =
    phase_trim - trim_one_signed;
wire signed [ADDR_WIDTH:0] trim_increment_target =
    (trim_incremented < trim_upper_limit) ?
    trim_incremented : trim_upper_limit;
wire signed [ADDR_WIDTH:0] trim_decrement_target =
    (trim_decremented > trim_lower_limit) ?
    trim_decremented : trim_lower_limit;

// Fractional playback clock.  A captured waveform contains approximately
// period_samples/4 points and must be traversed twice in period_samples input
// clocks.  The old fixed two-clocks-per-point scheme was exact only when the
// input period was divisible by four; otherwise the input crossing truncated
// playback once per period and produced a visible discontinuity above 80 kHz.
wire [ADDR_WIDTH+1:0] playback_phase_sum =
    {1'b0, playback_phase_accum} +
    {1'b0, completed_length, 1'b0};
wire playback_advance =
    completed_valid &&
    (period_samples != {ADDR_WIDTH{1'b0}}) &&
    (playback_phase_sum >= {2'b00, period_samples});
wire playback_wrap =
    completed_valid && !input_crossing && playback_advance &&
    ((playback_address + 1'b1) >= completed_length);

// Anticipate a bank swap at the crossing edge.  This prevents port B from
// reading the new write bank during the one clock in which the bank bits swap.
assign ram_read_address =
    (input_crossing && capture_active) ?
        {write_bank, bank_switch_start_address} :
        {read_bank, trimmed_read_address[BANK_ADDR_WIDTH-1:0]};

Mode3_Buffer u_mode3_buffer (
    .clka  (clk),
    .wea   (ram_write_enable),
    .addra (ram_write_address),
    .dina  (data_in),
    .clkb  (clk),
    .addrb (ram_read_address),
    .doutb (ram_read_data)
);

initial begin
    write_bank       = 1'b0;
    read_bank        = 1'b0;
    capture_active   = 1'b0;
    completed_valid  = 1'b0;
    write_address    = {ADDR_WIDTH{1'b0}};
    completed_length = {ADDR_WIDTH{1'b0}};
    playback_address = {ADDR_WIDTH{1'b0}};
    playback_phase_accum = {(ADDR_WIDTH+1){1'b0}};
    phase_trim       = {(ADDR_WIDTH+1){1'b0}};
    phase_trim_pending = {(ADDR_WIDTH+1){1'b0}};
    trim_update_waiting = 1'b0;
    phase_lock_confirm_count = 3'd0;
    phase_unlock_count = 2'd0;
    downsample_count = 2'd0;

    input_armed      = 1'b0;
    feedback_armed   = 1'b0;
    input_center     = MID_SCALE;
    input_min        = {DATA_WIDTH{1'b1}};
    input_max        = {DATA_WIDTH{1'b0}};
    feedback_center  = MID_SCALE;
    feedback_min     = {DATA_WIDTH{1'b1}};
    feedback_max     = {DATA_WIDTH{1'b0}};
    input_filter_acc =
        {MID_SCALE, {FILTER_SHIFT{1'b0}}};
    feedback_filter_acc =
        {MID_SCALE, {FILTER_SHIFT{1'b0}}};
    reference_age    = {ADDR_WIDTH{1'b0}};
    phase_locked     = 1'b0;
    data_out         = MID_SCALE;
end

always @(posedge clk) begin
    // Shift-only first-order smoothing for robust zero-crossing timing.
    input_filter_acc <=
        input_filter_acc -
        (input_filter_acc >> FILTER_SHIFT) +
        {{FILTER_SHIFT{1'b0}}, data_in};
    feedback_filter_acc <=
        feedback_filter_acc -
        (feedback_filter_acc >> FILTER_SHIFT) +
        {{FILTER_SHIFT{1'b0}}, feedback_data};

    // Adaptive input and feedback crossing detectors.
    if (input_filtered < input_min)
        input_min <= input_filtered;
    if (input_filtered > input_max)
        input_max <= input_filtered;
    if (feedback_filtered < feedback_min)
        feedback_min <= feedback_filtered;
    if (feedback_filtered > feedback_max)
        feedback_max <= feedback_filtered;

    if (input_filtered <= input_low_threshold)
        input_armed <= 1'b1;
    else if (input_crossing)
        input_armed <= 1'b0;

    if (feedback_filtered <= feedback_low_threshold)
        feedback_armed <= 1'b1;
    else if (feedback_crossing)
        feedback_armed <= 1'b0;

    if (input_crossing) begin
        reference_age <= {ADDR_WIDTH{1'b0}};
        input_center <=
            ({1'b0, input_min} + {1'b0, input_max}) >> 1;
        feedback_center <=
            ({1'b0, feedback_min} + {1'b0, feedback_max}) >> 1;
        input_min <= input_filtered;
        input_max <= input_filtered;
        feedback_min <= feedback_filtered;
        feedback_max <= feedback_filtered;
    end
    else if (!(&reference_age)) begin
        reference_age <= reference_age + 1'b1;
    end

    if (!enable || !period_locked) begin
        write_bank <= 1'b0;
        read_bank <= 1'b0;
        capture_active <= 1'b0;
        completed_valid <= 1'b0;
        write_address <= {ADDR_WIDTH{1'b0}};
        completed_length <= {ADDR_WIDTH{1'b0}};
        playback_address <= {ADDR_WIDTH{1'b0}};
        playback_phase_accum <= {(ADDR_WIDTH+1){1'b0}};
        phase_trim <= {(ADDR_WIDTH+1){1'b0}};
        phase_trim_pending <= {(ADDR_WIDTH+1){1'b0}};
        trim_update_waiting <= 1'b0;
        phase_lock_confirm_count <= 3'd0;
        phase_unlock_count <= 2'd0;
        downsample_count <= 2'd0;
        phase_locked <= 1'b0;
        data_out <= MID_SCALE;
    end
    else begin
        // Capture one complete period at one quarter of the 50 MHz sample
        // rate.  A crossing closes the old bank and writes address zero of the
        // new bank in the same clock.
        if (input_crossing) begin
            if (!capture_active) begin
                capture_active <= 1'b1;
                write_address <= {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
            end
            else begin
                completed_length <= write_address;
                if ((write_address > completed_length + 16'd1) ||
                    ((write_address + 16'd1) < completed_length)) begin
                    phase_trim <= {(ADDR_WIDTH+1){1'b0}};
                    phase_trim_pending <= {(ADDR_WIDTH+1){1'b0}};
                    trim_update_waiting <= 1'b0;
                    phase_lock_confirm_count <= 3'd0;
                    phase_unlock_count <= 2'd0;
                    phase_locked <= 1'b0;
                end
                read_bank <= write_bank;
                completed_valid <= 1'b1;
                write_bank <= ~write_bank;
                write_address <= {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
                playback_address <= {ADDR_WIDTH{1'b0}};
                playback_phase_accum <= {(ADDR_WIDTH+1){1'b0}};
                // Port B has one clock of read latency.  The crossing edge
                // prefetches the first address of the completed bank.
            end
            downsample_count <= 2'd0;
        end
        else if (capture_active) begin
            if (downsample_count == 2'd3) begin
                downsample_count <= 2'd0;
                if (write_address < BANK_DEPTH)
                    write_address <= write_address + 1'b1;
            end
            else begin
                downsample_count <= downsample_count + 1'b1;
            end
        end

        // Traverse the captured bank twice per measured input period.  The
        // phase accumulator distributes the fractional clock remainder over
        // the waveform instead of concentrating it into a boundary jump.
        if (completed_valid) begin
            if (input_crossing) begin
                playback_address <= {ADDR_WIDTH{1'b0}};
                playback_phase_accum <= {(ADDR_WIDTH+1){1'b0}};
            end
            else begin
                data_out <= ram_read_data;
                if (playback_advance) begin
                    playback_phase_accum <=
                        playback_phase_sum - {2'b00, period_samples};
                    if ((playback_address + 1'b1) >= completed_length) begin
                        playback_address <= {ADDR_WIDTH{1'b0}};
                        // Commit exactly one requested address correction at
                        // a waveform boundary, then allow a new measurement.
                        if (trim_update_waiting) begin
                            phase_trim <= phase_trim_pending;
                            trim_update_waiting <= 1'b0;
                        end
                    end
                    else begin
                        playback_address <= playback_address + 1'b1;
                    end
                end
                else begin
                    playback_phase_accum <= playback_phase_sum[ADDR_WIDTH:0];
                end
            end
        end
        else begin
            data_out <= MID_SCALE;
        end

        // Closed-loop fine phase adjustment of the stable playback address.
        if (completed_valid && selected_feedback_crossing && !input_crossing &&
            !playback_wrap && !trim_update_waiting &&
            (half_period != {ADDR_WIDTH{1'b0}})) begin
            if (phase_locked) begin
                phase_lock_confirm_count <= phase_lock_confirm_count;
                if (phase_inside_release_band) begin
                    phase_unlock_count <= 2'd0;
                    phase_locked <= 1'b1;
                end
                else if (phase_unlock_count >= 2'd2) begin
                    phase_trim_pending <=
                        trim_increment_direction ?
                        trim_increment_target : trim_decrement_target;
                    trim_update_waiting <= 1'b1;
                    phase_lock_confirm_count <= 3'd0;
                    phase_unlock_count <= 2'd0;
                    phase_locked <= 1'b0;
                end
                else begin
                    phase_unlock_count <= phase_unlock_count + 1'b1;
                    phase_locked <= 1'b1;
                end
            end
            else if (phase_inside_acquire_band) begin
                phase_unlock_count <= 2'd0;
                if (phase_lock_confirm_count >= 3'd3) begin
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
                phase_trim_pending <=
                    trim_increment_direction ?
                    trim_increment_target : trim_decrement_target;
                trim_update_waiting <= 1'b1;
                phase_lock_confirm_count <= 3'd0;
                phase_unlock_count <= 2'd0;
                phase_locked <= 1'b0;
            end
        end
    end
end

endmodule
