import pygame
import pygame.gfxdraw
import pymunk

import time
import random
import sys

pygame.init()
pygame.key.set_repeat(200, 50) # so multiple KEYDOWN events get sent ( ie holding down a key works )

#=========CONTROLS==========
# MOVE: WASD
# ZOOM IN/OUT: Q/E
# PAUSE SIMULATION: P
# INCREASE/DECREASE dt: L/K
# ADD A BODY TO MOUSE POSITION: RMB

class Constants: # Change stuff about the simulation here

    # body related stuff
    body_radius = 4
    body_mass = 1e6
    body_elasticity = 0.2
    body_friction = 0.8

    # display/pygame stuff
    fullscreen = True
    width = 1200
    height = 800

    move_by = 20 # pixels
    zoom_by = 0.01 # percent
    min_zoom = 1/(2*body_radius) # point when round(body_radius*zoom_percent) is 0
    max_zoom = 100*zoom_by
    starting_zoom = 0.4
    increase_dt_by = 0.01

    # sim related stuff
    bodies_N = 200 # starting body amount

    G = 6e-5
    collision_iterations = 20 # Increasing this doesn't significantly help with errors
    minimum_force = 10  # minimum force that is registered (in N)
    gravity_min_dist = 2*body_radius # min distance squared where gravity will be applied
    gravity_min_dist_sqrd = gravity_min_dist**2
    gravity_max_dist = (G*body_mass*body_mass/minimum_force)**0.5 # max distance squared where gravity will be applied
    gravity_max_dist_sqrd = gravity_max_dist**2
    per_frame_gravity = 4 # recalculate gravity every {per_frame_gravity} frames
    dt = 0.1


    # using a spatial hash didn't really improve performance, at least from what I tried
    use_spatial_hash = False
    spatial_hash_dim = 4*body_radius
    spatial_hash_count = bodies_N*10

    # FUCKING ANGULAR VELOCITY
    # Main problem of the simulation, colliding bodies seem to increase their angular velocity continuously
    # Haven't found a fix yet, as I believe its due to numerical errors ( increasing collision iterations doesn't help
    # as the problem persists even if they're cranked to 100-200 )
    # also increasing dt past ~0.15 results in lots of error
    # Only thing I could come up with are these
    dampen_angular_velocity = True # dampen angular velocity every collision, so it eventually goes to 0
    angular_dampening = 0.95
    cap_angular_velocity = True # cap the angular velocity a body can have
    angular_cap = 0.1


class CustomBody(pymunk.Body): # for now its useless, later I may use it for adding temperature
    def __init__(self, mass):
        moment = pymunk.moment_for_circle(mass, 0, Constants.body_radius)
        super().__init__(mass, moment)

def funky_gravity(bodies):
    # This monstrosity is my attempt at using list comprehensions for gravity force calculations, by abusing the := operator
    # which as it turns out is REALLY worse than the other method ( didn't continue it for this reason )

    forces = [diff_vec.normalized()*Constants.G*b1.mass*b2.mass/dist_sqrd for b2 in bodies for b1 in bodies if
              (dist_sqrd := (diff_vec := b2.position - b1.position).dot(diff_vec)) > Constants.gravity_min_dist_sqrd and
              dist_sqrd < Constants.gravity_max_dist_sqrd]


class Simulator:
    def __init__(self):

        # physics things
        self.is_paused = False  # controls only the physics sim, not drawing

        self.space = pymunk.Space()
        self.bodies = []
        self.N = 0
        self.dt = Constants.dt
        self.fps = 0
        self.sim_time = 0
        self.real_time = 0

        self.gravity_func = self._basic_gravity # for if I ever implement Barnes-Hut or any other method

        # display things
        if Constants.fullscreen:
            self._display = pygame.display.set_mode(flags=pygame.FULLSCREEN)
        else:
            self._display = pygame.display.set_mode((Constants.width, Constants.height))
        self._window_size_vec = pymunk.Vec2d(self._display.get_width(), self._display.get_height())

        self.position_displacement_vec = pymunk.Vec2d(0, 0)  # change the origin of the simulation
        self.zoom_percent = Constants.starting_zoom  # zoom in and out
        self.body_display_radius = round(Constants.body_radius*self.zoom_percent) # used in _update_display for bodies, updated every zoom event

        self.font = pygame.font.SysFont('freesans', 15)
        self.draw_UI = True

        self.draw_func = self._gfxdraw_bodies

        self.init_sim()

    def get_display_position_from_pymunk_position(self, position):
        # Thank stack overflow, everything except for that + self.position_displacement_vec is for zooming

        # TODO: since position*self.zoom_percent is basically the only variable parameter, maybe the majority of this can be cached
        # TODO: with the cached value changing when either zoom_percent or position_displacement_vec changes
        return (position - self._window_size_vec/2 + self.position_displacement_vec)*self.zoom_percent + self._window_size_vec/2

    def get_pymunk_position_from_display_position(self, position):
        # just the inverse of the get_display_position_from_pymunk_position function
        return (position - self._window_size_vec/2)/self.zoom_percent + self._window_size_vec/2 - self.position_displacement_vec

    def _update_body_display_radius(self):
        self.body_display_radius = round(Constants.body_radius*self.zoom_percent)

    def _custom_post_solve(self, arbiter, space, data):  # JUST FOR HANDLING ANGULAR VELOCITY
        # this function has next to no impact on performance, so don't try to optimize it
        for shape in arbiter.shapes:
            if Constants.dampen_angular_velocity:
                shape.body.angular_velocity *= Constants.angular_dampening
            if Constants.cap_angular_velocity:
                if shape.body.angular_velocity >= 0:
                    shape.body.angular_velocity = min(shape.body.angular_velocity, Constants.angular_cap)
                else:
                    shape.body.angular_velocity = max(shape.body.angular_velocity, Constants.angular_cap)

    def init_sim(self):

        # setup space
        self.space.gravity = (0, 0)
        self.space.iterations = Constants.collision_iterations

        handler = self.space.add_collision_handler(1, 1)
        handler.post_solve = self._custom_post_solve
        if Constants.use_spatial_hash:
            self.space.use_spatial_hash(
                dim=Constants.spatial_hash_dim,
                count=Constants.spatial_hash_count
            )

        # make all the bodies
        for _ in range(Constants.bodies_N):
            # coordinates of body, based on screen dimensions
            x = int(random.triangular(0, self._window_size_vec.x, self._window_size_vec.x//2))
            y = int(random.triangular(0, self._window_size_vec.y, self._window_size_vec.y//2))

            position = self.get_pymunk_position_from_display_position(pymunk.Vec2d(x, y)) # to work with different starting zoom levels
            self.add_body(position)

    def add_body(self, position):
        body = CustomBody(Constants.body_mass)
        body.position = position

        shape = pymunk.Circle(body, radius=Constants.body_radius)
        shape.elasticity = Constants.body_elasticity
        shape.friction = Constants.body_friction
        shape.collision_type = 1

        self.bodies.append(body)
        self.space.add(body, shape)
        self.N += 1

    def _basic_gravity(self, bodies):
        # This gives the most performance out of all the things I tried
        # using numpy doesn't help at all from what I tested, it even makes the performances worse by a lot

        forces = [pymunk.Vec2d.zero()]*self.N # this I think is faster than [pymunk.Vec2d.zero() for _ in range(len(bodies))]

        # since the mass of all bodies is the same, this can be cached, although it doesn't have a major impact on performance
        G_mass_sqrd = Constants.G*Constants.body_mass*Constants.body_mass

        for i in range(self.N):
            body1 = bodies[i]
            sum = pymunk.Vec2d.zero()
            position1 = body1.position # caching this has a pretty substantial impact on performance
            for j in range(i, self.N):
                # both getting the distance (with length) or squared distance (with dot) and
                # changing this function accordingly have the same impact on performance surprisingly
                # getting the distance is simpler and more clear though
                diff_vec = bodies[j].position - position1
                dist = diff_vec.length
                if Constants.gravity_min_dist <= dist <= Constants.gravity_max_dist:

                    F_dir = diff_vec/dist
                    F_mag = G_mass_sqrd/(dist*dist)
                    F = F_mag*F_dir

                    sum += F
                    forces[j] -= F

            # this instead of at local point helps a bit with errors
            body1.apply_force_at_world_point(forces[i] + sum, position1)



    def _draw_UI(self):

        # coordinates here are display coordinates

        left_height_offset = 10

        fps_text = self.font.render('FPS: {}'.format(self.fps), True, (255, 255, 255))
        fps_text.set_colorkey((0, 0, 0))
        self._display.blit(fps_text, fps_text.get_rect().move(10, left_height_offset))


        left_height_offset += 5 + fps_text.get_height()
        body_N_text = self.font.render('{} Bodies'.format(self.N), True, (255, 255, 255))
        body_N_text.set_colorkey((0, 0, 0))
        self._display.blit(body_N_text, body_N_text.get_rect().move(10, left_height_offset))

        left_height_offset += 5 + body_N_text.get_height()

        dt_text = self.font.render('dt: {}s, Sim Time: {}s, Real Time: {}s'.format(self.dt, self.sim_time, self.real_time), True, (255, 255, 255))
        dt_text.set_colorkey((0, 0, 0))
        self._display.blit(dt_text, dt_text.get_rect().move(10, left_height_offset))

        left_height_offset += 5 + dt_text.get_height()

        right_height_offset = 5
        zoom_text = self.font.render('zoom: {} %'.format(int(self.zoom_percent*100)), True, (255, 255, 255))
        zoom_text.set_colorkey((0, 0, 0))
        self._display.blit(zoom_text, zoom_text.get_rect().move(self._window_size_vec.x - zoom_text.get_width() - 10, right_height_offset))

        right_height_offset += 5 + zoom_text.get_height()

    def _draw_bodies(self):
        # This helps a lot with performance, mainly bc its drawing simple circles instead of force arrows too
        # Also, only way to add zooming and panning to the sim

        for i in range(self.N):
            body = self.bodies[i]
            position = self.get_display_position_from_pymunk_position(body.position)

            # don't draw things outside the screen
            if (position.x > self._window_size_vec.x or position.x < 0 or
                position.y > self._window_size_vec.y or position.y < 0):
                continue

            pygame.draw.circle(self._display, (255, 255, 255), position, self.body_display_radius)

    def _gfxdraw_bodies(self):
        # same as _draw_bodies but optimised to use gfxdraw instead of draw, which is a bit faster
        # the filled circles of gfxdraw are not good though

        for i in range(self.N):
            body = self.bodies[i]
            x, y = self.get_display_position_from_pymunk_position(body.position).int_tuple

            # don't draw things outside the screen
            if (x > self._window_size_vec.x or x < 0 or
                y > self._window_size_vec.y or y < 0):
                continue

            pygame.gfxdraw.filled_circle(self._display, x, y, self.body_display_radius, (255, 255,255))

    def start(self):

        total_frames = 0
        current_frames = 0

        t_start = time.time()
        t0 = t_start

        running = True
        while running:
            current_frames += 1
            total_frames += 1

            if time.time() - t0 >= 1:
                self.fps = current_frames
                self.real_time += 1
                current_frames = 0
                t0 = time.time()


            for event in pygame.event.get():
                if (event.type == pygame.QUIT or
                    (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE)):
                    running = False

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    # 1 is left click, 3 is right click
                    if event.button == 1:
                        position = self.get_pymunk_position_from_display_position(pygame.mouse.get_pos())
                        self.add_body(position)

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_p:
                        self.is_paused = not self.is_paused

                    elif event.key == pygame.K_w:
                        self.position_displacement_vec += (0, Constants.move_by)
                    elif event.key == pygame.K_a:
                        self.position_displacement_vec += (Constants.move_by, 0)
                    elif event.key == pygame.K_s:
                        self.position_displacement_vec += (0, -Constants.move_by)
                    elif event.key == pygame.K_d:
                        self.position_displacement_vec += (-Constants.move_by, 0)

                    elif event.key == pygame.K_q and self.zoom_percent - Constants.zoom_by > Constants.min_zoom:
                        self.zoom_percent -= Constants.zoom_by
                        self._update_body_display_radius()
                    elif event.key == pygame.K_e and self.zoom_percent + Constants.zoom_by < Constants.max_zoom:
                        self.zoom_percent += Constants.zoom_by
                        self._update_body_display_radius()

                    elif event.key == pygame.K_k:
                        self.dt = round(self.dt - Constants.increase_dt_by, 3)
                    elif event.key == pygame.K_l:
                        self.dt = round(self.dt + Constants.increase_dt_by, 3)

                    elif event.key == pygame.K_h:
                        self.draw_UI = not self.draw_UI

            if not self.is_paused:
                if total_frames % Constants.per_frame_gravity == 0:
                    self.gravity_func(self.bodies)
                self.space.step(self.dt)

            self._display.fill((0, 0, 0))

            self.draw_func()
            if self.draw_UI:
                self._draw_UI()

            pygame.display.update()

            self.sim_time = round(self.sim_time + self.dt, 3)

        return 0

if __name__ == "__main__":
    sim = Simulator()
    sys.exit(sim.start())