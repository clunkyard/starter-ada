-- A robot on the Clunkyard core's Ada binding. The core owns tracking,
-- geometry and protocol handling; these procedures choose the tactics.
-- Clunkyard robot API 2.0.0: one core under every language, the same
-- answers everywhere. Facts (a call shows its arguments):
-- match: mode ruleset control robots is_3d tick_rate max_ticks
--   robot_id(slot)
-- clock: tick seconds remaining_ticks
-- arena: size gravity terrain_kind ground_height(x y)
-- me: health position velocity heading pitch roll angular_velocity
--   attitude grounded altitude applied
-- battery: charge capacity regen sustainable_drive compute
--   drive_cost(fraction)
-- drive: hover_lift
-- gun: aim_now cooldown_ticks spread max_power damage_scale
--   can_fire(power reserve) shot_speed(power) shot_cost(power)
--   reload_ticks(power) shot_damage(power) power_for_speed(speed)
--   power_for_flight_time(range seconds) lead(contact power)
--   lead_point(position velocity power)
-- scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
-- sensors: contacts nearest cues contact(slot) position_of(contact)
-- tracker: cue nearest_bearing_rate velocity(slot) position(slot)
--   last_seen(slot)
-- nav: zone zone.margin(point) zone.contains(point margin)
--   zone.distance_from(point) storm storm.margin(point)
--   storm.contains(point margin) storm.distance_from(point) hill
--   hill.margin(point) hill.contains(point margin)
--   hill.distance_from(point) pickups nearest_pickup(kind)
--   field_distance(a b)
-- score: me best armour
-- angles: normalize(degrees) angle_diff(target current)
--   bearing_to(from to) elevation_to(from to) direction(heading pitch)
--   distance(a b) clamp(value low high)
-- raw: number(path) text(path) has(path) flag(path) count(path) line()
-- Controls: drive.set(throttle turn pitch_turn)
--   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
--   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
--   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
--   status.set(text value) raw.extra(json)
-- Every control argument is optional; a second call keeps what the first
-- set. Absent facts are your language's none. Names are snake_case here;
-- your language cases them its own way. Angles are degrees from the +x
-- axis; pitch is positive up.
-- Manual: https://clunkyard.com/manual#sdk
with Clunkyard;use Clunkyard;
with Interfaces;use Interfaces;
with Interfaces.C;use Interfaces.C;
with System;
procedure Bot is
   type Query is access function(Handle:System.Address;Value:access Real)return I32 with Convention=>C;
   Robot:aliased System.Address:=System.Null_Address;
   Code: I32;
   Event:aliased I32;
   Name:aliased constant String:="clunkyard-ada";
   Version:aliased constant String:="2.0.0";
   function Get(Handle:System.Address;Read:Query)return Real is
      Value:aliased Real:=0.0;Status:I32;
   begin Status:=Read(Handle,Value'Access);return Value;end;
   function Steer(Towards,Current:Real)return Real is
      Delta_Value,Value:aliased Real:=0.0;Status:I32;
   begin Status:=cy_angles_angle_diff(Towards,Current,Delta_Value'Access);Status:=cy_angles_clamp(Delta_Value/45.0,-1.0,1.0,Value'Access);return Value;end;
   procedure Drive(Handle:System.Address;Throttle,Turning,Pitching:Real)is Status:I32;
   begin Status:=cy_drive_throttle(Handle,Throttle);Status:=cy_drive_turn(Handle,Turning);Status:=cy_drive_pitch_turn(Handle,Pitching);end;
   procedure Scan(Handle:System.Address;Heading,Arc,Pitch:Real)is Status:I32;
   begin Status:=cy_scanner_heading(Handle,Heading);Status:=cy_scanner_arc(Handle,Arc);Status:=cy_scanner_pitch(Handle,Pitch);end;
   procedure Show(Handle:System.Address;Text:String)is Status:I32;
   begin Status:=cy_status_text(Handle,Text'Address,U32(Text'Length));end;
   procedure Show(Handle:System.Address;Text:String;Value:Real)is Status:I32;
   begin Show(Handle,Text);Status:=cy_status_value(Handle,Value);end;
   procedure Turn(Handle:System.Address)is
      Position,Gravity:aliased Cy_Vec3;
      Target:aliased Cy_Contact;Cue:aliased Cy_Cue;Last:aliased Cy_Scan;Aim:aliased Cy_Solution;Now:aliased Cy_Direction;
      Storm:aliased Cy_Storm;Zone:aliased Cy_Zone;
      Status:I32;Tick,Is_3d,Can_Fire:aliased I32;
      Heading,Pitch,Throttle,Error,Across,Up,Climb:aliased Real;
      Power,Delta_Value,Bearing,Distance,Elevation,Arc,Sweep,Margin,Away,Control,Turning,Pitching:aliased Real;
      Free_Flight:Boolean;
   begin
      Status:=cy_me_position(Handle,Position'Access);Status:=cy_arena_gravity(Handle,Gravity'Access);Status:=cy_scanner_last(Handle,Last'Access);
      Heading:=Get(Handle,cy_me_heading'Access);Pitch:=Get(Handle,cy_me_pitch'Access);
      Status:=cy_clock_tick(Handle,Tick'Access);Status:=cy_match_is_3d(Handle,Is_3d'Access);Free_Flight:=Is_3d/=0 and then Gravity.Z=0.0;
      Status:=cy_drive_brake(Handle,0.2);
      if cy_sensors_nearest(Handle,Target'Access)=CY_OK then
         Throttle:=0.2;if Target.Range_Value>350.0 then Throttle:=0.7;elsif Target.Range_Value<200.0 then Throttle:=-0.4;end if;
         Drive(Handle,Throttle,Steer(Target.Bearing,Heading),Steer(Target.Elevation,Pitch));Scan(Handle,Target.Bearing,20.0,Target.Elevation);
         Status:=cy_gun_power_for_flight_time(Handle,Target.Range_Value,1.0,Power'Access);
         if cy_gun_lead(Handle,Target'Access,Power,Aim'Access)/=CY_OK then Aim.Heading:=Target.Bearing;Aim.Pitch:=Target.Elevation;end if;
         Status:=cy_gun_aim_heading(Handle,Aim.Heading);Status:=cy_gun_aim_pitch(Handle,Aim.Pitch);
         Status:=cy_gun_aim_now(Handle,Now'Access);Status:=cy_angles_angle_diff(Aim.Heading,Now.Heading,Delta_Value'Access);Status:=cy_gun_can_fire(Handle,Power,10.0,Can_Fire'Access);
         if abs Delta_Value<3.0 and then abs(Aim.Pitch-Now.Pitch)<3.0 and then Can_Fire/=0 then Status:=cy_gun_fire(Handle);Status:=cy_gun_power(Handle,Power);end if;
         Show(Handle,"TRACK",Target.Range_Value);
      elsif cy_tracker_cue(Handle,Cue'Access)=CY_OK then
         Status:=cy_angles_bearing_to(Position'Access,Cue.Position'Access,Bearing'Access);Status:=cy_angles_distance(Position'Access,Cue.Position'Access,Distance'Access);
         Elevation:=0.0;if Free_Flight then Status:=cy_angles_elevation_to(Position'Access,Cue.Position'Access,Elevation'Access);end if;
         Error:=Real'Min(Cue.Bearing_Error,30.0);Across:=Real(Tick mod 5-2)/2.0*Error;Up:=0.0;if Free_Flight then Up:=Real((Tick/5)mod 3-1)*Error/2.0;end if;
         Drive(Handle,0.7,Steer(Bearing,Heading),Steer(Elevation,Pitch));Status:=cy_scanner_arc_for_reach(Handle,Distance+100.0,Arc'Access);
         Scan(Handle,Bearing+Across,Arc,Elevation+Up);Show(Handle,"CUE",Distance);
      else
         Status:=cy_angles_normalize(Last.Heading+36.0,Sweep'Access);Up:=0.0;if Free_Flight then Up:=Real((Tick/10)mod 3-1)*35.0;end if;
         Drive(Handle,0.5,0.33,Steer(0.0,Pitch));Scan(Handle,Sweep,40.0,Up);Show(Handle,"SEARCH");
      end if;
      if Get(Handle,cy_battery_charge'Access)<Get(Handle,cy_battery_capacity'Access)/4.0 then Status:=cy_drive_throttle(Handle,Get(Handle,cy_drive_get_throttle'Access)*0.5);end if;
      if cy_nav_storm(Handle,Storm'Access)=CY_OK and then cy_nav_storm_margin(Handle,Position'Access,Margin'Access)=CY_OK and then Margin> -60.0 then
         Status:=cy_angles_bearing_to(Storm.Center'Access,Position'Access,Away'Access);Status:=cy_angles_angle_diff(Away,Heading,Delta_Value'Access);
         Status:=cy_angles_clamp(Delta_Value/30.0,-1.0,1.0,Control'Access);Status:=cy_drive_throttle(Handle,0.8);Status:=cy_drive_turn(Handle,Control);
      end if;
      if cy_nav_zone(Handle,Zone'Access)=CY_OK and then cy_nav_zone_margin(Handle,Position'Access,Margin'Access)=CY_OK and then Margin<60.0 then
         Status:=cy_angles_bearing_to(Position'Access,Zone.Center'Access,Bearing'Access);Climb:=0.0;if Zone.Shape=CY_SHAPE_SPHERE then Status:=cy_angles_elevation_to(Position'Access,Zone.Center'Access,Climb'Access);end if;
         Status:=cy_angles_angle_diff(Bearing,Heading,Delta_Value'Access);Status:=cy_angles_clamp(Delta_Value/30.0,-1.0,1.0,Turning'Access);
         Status:=cy_angles_clamp((Climb-Pitch)/30.0,-1.0,1.0,Pitching'Access);Drive(Handle,0.8,Turning,Pitching);
      end if;
   end;
begin
   Code:=cy_open(2,0,Name'Address,Name'Length,Version'Address,Version'Length,Robot'Access);
   if Code<0 then return;end if;
   loop
      Code:=cy_session_next(Robot,Event'Access);
      if Code=CY_E_INTERNAL then Code:=cy_session_send(Robot);exit when Code<0;
      else
         exit when Code/=CY_OK or else Event=CY_EVENT_END;
         if Event=CY_EVENT_STATE then Turn(Robot);end if;
         Code:=cy_session_send(Robot);exit when Code<0;
      end if;
   end loop;
   cy_close(Robot);
end Bot;
