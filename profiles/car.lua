-- Car profile

api_version = 0

local find_access_tag = require("lib/access").find_access_tag
local Set = require('lib/set')
local Sequence = require('lib/sequence')
local Handlers = require("lib/handlers")


-- set profile properties
properties.u_turn_penalty                  = 20
properties.traffic_signal_penalty          = 2
properties.max_speed_for_map_matching      = 180/3.6 -- 180kmph -> m/s
properties.use_turn_restrictions           = true
properties.continue_straight_at_waypoint   = true
properties.left_hand_driving               = false

local profile = {
  default_mode      = mode.driving,
  default_speed     = 10,
  oneway_handling   = true,
  
  side_road_speed_multiplier = 0.8,
  turn_penalty               = 7.5,
  speed_reduction            = 0.8,
  
  -- Note: this biases right-side driving.
  -- Should be inverted for left-driving countries.
  turn_bias   = properties.left_hand_driving and 1/1.075 or 1.075,

  -- a list of suffixes to suppress in name change instructions
  suffix_list = {
    'N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW', 'North', 'South', 'West', 'East'
  },
  
  barrier_whitelist = Set {
    'cattle_grid',
    'border_control',
    'checkpoint',
    'toll_booth',
    'sally_port',
    'gate',
    'lift_gate',
    'no',
    'entrance'
  },

  access_tag_whitelist = Set {
    'yes',
    'motorcar',
    'motor_vehicle',
    'vehicle',
    'permissive',
    'designated',
    'destination'
  },

  access_tag_blacklist = Set {
    'no',
    'private',
    'agricultural',
    'forestry',
    'emergency',
    'psv',
    'delivery'
  },

  access_tags_hierarchy = Sequence {
    'motorcar',
    'motor_vehicle',
    'vehicle',
    'access'
  },

  service_tag_forbidden = Set {
    'emergency_access'
  },

  restrictions = Sequence {
    'motorcar',
    'motor_vehicle',
    'vehicle'
  },

  prefetch = Set {
    'highway',
    'route',
    'bridge'
  },
  
  avoid = Set {
    'area',
    'toll',
    'reversible',
    'impassable',
    'hov_lanes'
  },
  
  speeds = Sequence {
    highway = {
      motorway        = 90,
      motorway_link   = 45,
      trunk           = 85,
      trunk_link      = 40,
      primary         = 65,
      primary_link    = 30,
      secondary       = 55,
      secondary_link  = 25,
      tertiary        = 40,
      tertiary_link   = 20,
      unclassified    = 25,
      residential     = 25,
      living_street   = 10,
      service         = 15,
    }
  },

  service_speeds = {
    alley             = 5,
    parking           = 5,
    parking_aisle     = 5,
    driveway          = 5,
    ["drive-through"] = 5
  },
  
  route_speeds = {
    ferry = 5,
    shuttle_train = 10
  },

  bridge_speeds = {
    movable = 5
  },
  
  -- surface/trackype/smoothness
  -- values were estimated from looking at the photos at the relevant wiki pages

  -- max speed for surfaces
  surface_speeds = {
    asphalt = nil,    -- nil mean no limit. removing the line has the same effect
    concrete = nil,
    ["concrete:plates"] = nil,
    ["concrete:lanes"] = nil,
    paved = nil,

    cement = 80,
    compacted = 80,
    fine_gravel = 80,

    paving_stones = 60,
    metal = 60,
    bricks = 60,

    grass = 40,
    wood = 40,
    sett = 40,
    grass_paver = 40,
    gravel = 40,
    unpaved = 40,
    ground = 40,
    dirt = 40,
    pebblestone = 40,
    tartan = 40,

    cobblestone = 30,
    clay = 30,

    earth = 20,
    stone = 20,
    rocky = 20,
    sand = 20,

    mud = 10
  },

  -- max speed for tracktypes
  tracktype_speeds = {
    grade1 =  60,
    grade2 =  40,
    grade3 =  30,
    grade4 =  25,
    grade5 =  20
  },

  -- max speed for smoothnesses
  smoothness_speeds = {
    intermediate    =  80,
    bad             =  40,
    very_bad        =  20,
    horrible        =  10,
    very_horrible   =  5,
    impassable      =  0
  },

  -- http://wiki.openstreetmap.org/wiki/Speed_limits
  maxspeed_table_default = {
    urban = 50,
    rural = 90,
    trunk = 110,
    motorway = 130
  },

  -- List only exceptions
  maxspeed_table = {
    ["ch:rural"] = 80,
    ["ch:trunk"] = 100,
    ["ch:motorway"] = 120,
    ["de:living_street"] = 7,
    ["ru:living_street"] = 20,
    ["ru:urban"] = 60,
    ["ua:urban"] = 60,
    ["at:rural"] = 100,
    ["de:rural"] = 100,
    ["at:trunk"] = 100,
    ["cz:trunk"] = 0,
    ["ro:trunk"] = 100,
    ["cz:motorway"] = 0,
    ["de:motorway"] = 0,
    ["ru:motorway"] = 110,
    ["gb:nsl_single"] = (60*1609)/1000,
    ["gb:nsl_dual"] = (70*1609)/1000,
    ["gb:motorway"] = (70*1609)/1000,
    ["uk:nsl_single"] = (60*1609)/1000,
    ["uk:nsl_dual"] = (70*1609)/1000,
    ["uk:motorway"] = (70*1609)/1000,
    ["nl:rural"] = 80,
    ["nl:trunk"] = 100,
    ["none"] = 140
  }
}

function get_name_suffix_list(vector)
  for index,suffix in ipairs(profile.suffix_list) do
      vector:Add(suffix)
  end
end

function get_restrictions(vector)
  for i,v in ipairs(profile.restrictions) do
    vector:Add(v)
  end
end

function node_function (node, result)
  -- parse access and barrier tags
  local access = find_access_tag(node, profile.access_tags_hierarchy)
  if access then
    if profile.access_tag_blacklist[access] then
      result.barrier = true
    end
  else
    local barrier = node:get_value_by_key("barrier")
    if barrier then
      --  make an exception for rising bollard barriers
      local bollard = node:get_value_by_key("bollard")
      local rising_bollard = bollard and "rising" == bollard

      if not profile.barrier_whitelist[barrier] and not rising_bollard then
        result.barrier = true
      end
    end
  end

  -- check if node is a traffic light
  local tag = node:get_value_by_key("highway")
  if "traffic_signals" == tag then
    result.traffic_lights = true
  end
end

-- main entry point for processsing a way
function way_function(way, result)
  -- intermediate values used during processing
  local data = {}

  handlers = Sequence {
    -- to optimize processing, we should try to abort as soon as
    -- possible if the way is not routable, to avoid doing
    -- unnecessary work. this implies we should check things that
    -- commonly forbids access early, and handle complicated edge
    -- cases later.

    -- perform an quick initial check and abort if way is obviously
    -- not routable, e.g. because it does not have any of the key
    -- tags indicating routability
    'handle_tag_prefetch',

    -- set the default mode for this profile. if can be changed later
    -- in case it turns we're e.g. on a ferry
    'handle_default_mode',

    -- check various tags that could indicate that the way is not
    -- routable. this includes things like status=impassable,
    -- toll=yes and oneway=reversible
    'handle_blocked_ways',

    -- determine access status by checking our hierarchy of
    -- access tags, e.g: motorcar, motor_vehicle, vehicle
    'handle_access',

    -- check whether forward/backward directons are routable
    'handle_oneway',

    -- check whether forward/backward directons are routable
    'handle_destinations',

    -- check whether we're using a special transport mode
    'handle_ferries',
    'handle_movables',

    -- handle service road restrictions
    'handle_service',

    -- check high occupancy vehicle restrictions
    'handle_hov',

    -- compute speed taking into account way type, maxspeed tags, etc.
    'handle_speed',
    'handle_side_roads',
    'handle_surface',
    'handle_maxspeed',
    'handle_speed_scaling',
    'handle_alternating_speed',

    -- handle turn lanes and road classification, used for guidance
    'handle_turn_lanes',
    'handle_classification',

    -- handle various other flags
    'handle_roundabouts',
    'handle_startpoint',

    -- set name, ref and pronunciation
    'handle_names'
  }
  
  Handlers.run(handlers,way,result,data,profile)
end

function turn_function (angle)
  -- Use a sigmoid function to return a penalty that maxes out at turn_penalty
  -- over the space of 0-180 degrees.  Values here were chosen by fitting
  -- the function to some turn penalty samples from real driving.
  -- multiplying by 10 converts to deci-seconds see issue #1318
  local penalty = profile.turn_penalty
  local bias    = profile.turn_bias
  
  if angle>=0 then
    return 10 * penalty / (1 + 2.718 ^ - ((13 / bias) * angle/180 - 6.5*bias))
  else
    return 10 * penalty / (1 + 2.718 ^  - ((13 * bias) * - angle/180 - 6.5/bias))
  end
end
