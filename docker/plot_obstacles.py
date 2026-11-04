"""Shared East-horizontal/North-vertical scenario obstacle overlays."""
from matplotlib.patches import Circle


def draw_obstacles(ax, manifest):
    """Draw physical disks and enabled ground CBF clearance without changing limits."""
    config = manifest.get('control', {}).get('obstacle_cbf', {})
    clearance = float(config.get('clearance_m', 0)) if config.get('enabled', False) else 0.
    artists = []
    for index, circle in enumerate(manifest['map']['obstacles']['circles']):
        north, east = circle['center_m']
        radius = circle['radius_m']
        disk = Circle((east, north), radius, facecolor='#64748b', edgecolor='black',
                      linewidth=1.2, zorder=2, label='Obstacle' if index == 0 else '_nolegend_')
        ax.add_patch(disk)
        artists.append(disk)
        if clearance > 0:
            boundary = Circle((east, north), radius + clearance, fill=False,
                              edgecolor='#b91c1c', linestyle='--', linewidth=1.2,
                              zorder=2.5, label='Ground CBF clearance' if index == 0 else '_nolegend_')
            ax.add_patch(boundary)
            artists.append(boundary)
    return artists
