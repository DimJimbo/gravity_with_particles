import pygame
import pymunk
import pymunk.pygame_util

import time
import random
import sys
import multiprocessing as mp

pygame.init()
pygame.key.set_repeat(200, 50) # so multiple KEYDOWN events get sent ( ie holding down a key works )

#=========CONTROLS==========
# MOVE: WASD
# ZOOM IN/OUT: Q/E
# PAUSE SIMULATION: P

class Constants: # Change stuff about the simulation here

    # body related stuff
    body_radius = 4
    body_mass = 1e6
    body_elasticity = 0.8
    body_friction = 0.6

    # display/pygame stuff
    fullscreen = False
    width = 1200
    height = 800

    per_frame_draw = 1 # redraw screen every {per_frame_draw} frames
    move_by = 20 # pixels
    zoom_by = 0.05 # percent
    min_zoom = 1/(2*body_radius) # point when round(body_radius*zoom_percent) is 0
    max_zoom = 100*zoom_by
    starting_zoom = 1

    # sim related stuff
    bodies_N = 200 # starting body amount

    collision_iterations = 20 # Increasing this doesn't significantly help with errors
    gravity_min_dist_sqrd = (2*body_radius)**2 # min distance squared where gravity will be applied
    gravity_max_dist_sqrd = 600**2 # max distance squared where gravity will be applied
    per_frame_gravity = 5 # recalculate gravity every {per_frame_gravity} frames
    dt = 0.1
    G = 6e-5

    # using a spatial hash didn't really improve performance, at least from what I tried
    use_spatial_hash = False
    spatial_hash_dim = 4*body_radius
    spatial_hash_count = bodies_N*10

    # FUCKING ANGULAR VELOCITY
    # Main problem of the simulation, colliding bodies seem to increase their angular velocity continuously
    # Haven't found a fix yet, as I believe its due to numerical errors ( increasing collision iterations doesn't help
    # as the problem persists unchanged even if they're cranked to 100-200 ), only thing to be done are these
    dampen_angular_velocity = True # dampen angular velocity every collision, so it eventually goes to 0
    angular_dampening = 0.95
    cap_angular_velocity = True # cap the angular velocity a body can have
    angular_cap = 0.1


class CustomBody(pymunk.Body): # for now its useless, later I may use it for adding temperature
    def __init__(self, mass):
        moment = pymunk.moment_for_circle(mass, 0, Constants.body_radius)
        super().__init__(mass, moment)

def custom_post_solve(arbiter, space, data): # JUST FOR HANDLING ANGULAR VELOCITY
    for shape in arbiter.shapes:
        if Constants.dampen_angular_velocity:
            shape.body.angular_velocity *= Constants.angular_dampening
        if Constants.cap_angular_velocity:
            if shape.body.angular_velocity >= 0:
                shape.body.angular_velocity = min(shape.body.angular_velocity, Constants.angular_cap)
            else:
                shape.body.angular_velocity = max(shape.body.angular_velocity, Constants.angular_cap)


def funky_gravity(bodies):
    # This monstrocity is my attempt at using list comprehensions for gravity force calculations, by abusing the := operator
    # which as it turns out is REALLY worse than the other method ( didn't continue it for this reason )

    forces = [diff_vec.normalized()*Constants.G*b1.mass*b2.mass/dist_sqrd for b2 in bodies for b1 in bodies if
              (dist_sqrd := (diff_vec := b2.position - b1.position).dot(diff_vec)) > Constants.gravity_min_dist_sqrd and
              dist_sqrd < Constants.gravity_max_dist_sqrd]


class Simulator:
    def __init__(self):

        if Constants.fullscreen:
            self._display = pygame.display.set_mode(flags=pygame.FULLSCREEN)
        else:
            self._display = pygame.display.set_mode((Constants.width, Constants.height))
        self._window_size_vec = pymunk.Vec2d(self._display.get_width(), self._display.get_height())

        self.space = pymunk.Space()
        self.bodies = []

        self.gravity_func = self._basic_gravity # for if I ever implement Barnes-Hut or any other method

        self.is_paused = False # controls only the physics sim, not drawing
        self.position_displacement_vec = pymunk.Vec2d(0, 0) # change the origin of the simulation
        self.zoom_percent = Constants.starting_zoom # zoom in and out

        self.init_sim()

    def get_display_position_from_pymunk_position(self, position):
        # Thank stack overflow, everything except for that + self.position_diplacement_vec is for zooming
        return (position - self._window_size_vec/2 + self.position_displacement_vec)*self.zoom_percent + self._window_size_vec/2
    def get_pymunk_position_from_display_position(self, position):
        # just the inverse of the one at the _update_display function
        return (position - self._window_size_vec/2)/self.zoom_percent + self._window_size_vec/2 - self.position_displacement_vec
    def init_sim(self):

        # setup space
        self.space.gravity = (0, 0)
        self.space.iterations = Constants.collision_iterations

        handler = self.space.add_collision_handler(1, 1)
        handler.post_solve = custom_post_solve
        if Constants.use_spatial_hash:
            self.space.use_spatial_hash(
                dim=Constants.spatial_hash_dim,
                count=Constants.spatial_hash_count
            )


        # make all the bodies
        for _ in range(Constants.bodies_N):
            # coordinates of body, based on screen dimensions
            x = random.randint(0, Constants.width)
            y = random.randint(0, Constants.height)

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

    def _basic_gravity(self, bodies):
        # This gives the most performance out of all the things I tried
        # using numpy doesn't help at all from what I tested, it even makes the performances worse by a lot

        forces = [pymunk.Vec2d.zero()]*len(bodies) # this I think is faster than [pymunk.Vec2d.zero() for _ in range(len(bodies))]

        # caching this actually has some impact on performance, not a lot tho
        mass_sqrd = Constants.body_mass*Constants.body_mass

        for i, body1 in enumerate(bodies):
            position1 = body1.position # caching this has a pretty substantial impact on performance

            for j, body2 in enumerate(bodies[i + 1:]):
                diff_vec = body2.position - position1
                dist_sqrd = diff_vec.dot(diff_vec)  # faster than b1.position.get_dist_sqrd(b2.position)

                if (dist_sqrd < Constants.gravity_min_dist_sqrd or
                        dist_sqrd > Constants.gravity_max_dist_sqrd):
                    continue

                F_dir = diff_vec.normalized()
                F_mag = Constants.G*mass_sqrd/dist_sqrd
                F = F_dir*F_mag

                forces[i] += F
                forces[i + 1 + j] -= F

            # this instead of at local point helps a bit with errors
            body1.apply_force_at_world_point(forces[i], position1)

        # for i, body1 in enumerate(self.bodies):
        #     # this instead of at local point helps a bit with errors
        #     body1.apply_force_at_world_point(forces[i], body1.position)
    def _update_display(self):
        # This helps a lot with performance, mainly bc its drawing simple circles instead of force arrows too
        # Also, only way to add zooming and panning to the sim

        self._display.fill((0, 0, 0))

        for body in self.bodies:

            position = self.get_display_position_from_pymunk_position(body.position)

            # don't draw things outside the screen
            if (position.x > self._window_size_vec.x or position.x < 0 or
                position.y > self._window_size_vec.y or position.y < 0):
                continue

            radius = round(Constants.body_radius*self.zoom_percent)
            pygame.draw.circle(self._display, (255, 255, 255), position, radius)

        pygame.display.update()

    def start(self):

        total_frames = 0
        fps = 0

        t_start = time.time()
        t0 = t_start

        running = True
        while running:
            fps += 1
            total_frames += 1

            if time.time() - t0 >= 1:
                print('fps:', fps)
                fps = 0
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
                    elif event.key == pygame.K_e and self.zoom_percent + Constants.zoom_by < Constants.max_zoom:
                        self.zoom_percent += Constants.zoom_by

            if not self.is_paused:
                if total_frames % Constants.per_frame_gravity == 0:
                    self.gravity_func(self.bodies)

                self.space.step(Constants.dt)

            if total_frames % Constants.per_frame_draw == 0:
                self._update_display()

        return 0

if __name__ == "__main__":
    sim = Simulator()
    sys.exit(sim.start())