-- Foot profile

api_version = 0

local find_access_tag = require("lib/access").find_access_tag
local Set = require('lib/set')
local Sequence = require('lib/sequence')
local Handlers = require("lib/handlers")


properties.traffic_signal_penalty        = 2
properties.u_turn_penalty                = 2
properties.max_speed_for_map_matching    = 40/3.6 -- kmph -> m/s
properties.use_turn_restrictions         = false
properties.continue_straight_at_waypoint = false


local walking_speed = 5

local profile = {
  default_mode      = mode.walking,
  default_speed     = walking_speed,
  oneway_handling   = 'specific',     -- respect 'oneway:foot' but not 'oneway'
  
  barrier_whitelist = Set {
    'cycle_barrier',
    'bollard',
    'entrance',
    'cattle_grid',
    'border_control',
    'toll_booth',
    'sally_port',
    'gate',
    'no',
    'block'
  },
  
  access_tag_whitelist = Set {
    'yes', 
    'foot', 
    'permissive', 
    'designated'  
  },
  
  access_tag_blacklist = Set {
    'no', 
    'private', 
    'agricultural', 
    'forestry', 
    'delivery'
  },

  access_tags_hierarchy = Sequence {
    'foot',
    'access'
  },

  restrictions = Sequence {
    'foot'
  },

  -- list of suffixes to suppress in name change instructions
  suffix_list = Set {
    'N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW', 'North', 'South', 'West', 'East'
  },
  
  prefetch = Set {
    'highway',
    'leisure',
    'route',
    'bridge',
    'man_made',
    'railway',
    'platform',
    'amenity',
    'public_transport'
  },

  avoid = Set {
    'impassable'
  },
  
  speeds = Sequence {
    highway = {
      primary         = walking_speed,
      primary_link    = walking_speed,
      secondary       = walking_speed,
      secondary_link  = walking_speed,
      tertiary        = walking_speed,
      tertiary_link   = walking_speed,
      unclassified    = walking_speed,
      residential     = walking_speed,
      road            = walking_speed,
      living_street   = walking_speed,
      service         = walking_speed,
      track           = walking_speed,
      path            = walking_speed,
      steps           = walking_speed,
      pedestrian      = walking_speed,
      footway         = walking_speed,
      pier            = walking_speed,
    },

    railway = {
      platform        = walking_speed
    },

    amenity = {
      parking         = walking_speed,
      parking_entrance= walking_speed
    },

    man_made = {
      pier            = walking_speed
    },
    
    leisure = {
      track           = walking_speed
    }
  },
  
  route_speeds = {
    ferry = 5
  },
  
  surface_speeds = {
    fine_gravel =   walking_speed*0.75,
    gravel =        walking_speed*0.75,
    pebblestone =   walking_speed*0.75,
    mud =           walking_speed*0.5,
    sand =          walking_speed*0.5
  }
}


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

  local handlers = Sequence {
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

    -- compute speed taking into account way type, maxspeed tags, etc.
    'handle_speed',
    'handle_surface',

    -- handle turn lanes and road classification, used for guidance
    'handle_classification',

    -- handle various other flags
    'handle_roundabouts',
    'handle_startpoint',

    -- set name, ref and pronunciation
    'handle_names'
  }
  
  Handlers.run(handlers,way,result,data,profile)
end
