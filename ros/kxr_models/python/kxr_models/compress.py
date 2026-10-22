import os
import os.path as osp
import tarfile
import tempfile


def make_tarfile(output_filename, source_dir=None):
    """Make tarfile

    The archive is written to a temporary file and renamed into place, so
    a reader never sees a partially written ``output_filename``.

    Parameters
    ----------
    output_filename : str
        output filename. If source_dir is None, output_filename is considered
        as the source_dir name.
    source_dir : str or None
        target directory to compress.

    Returns
    -------
    output_filename : str
        output filename
    """
    if source_dir is None:
        source_dir = output_filename
        output_filename = f"{source_dir}.tar.gz"
    fd, temp_path = tempfile.mkstemp(
        dir=osp.dirname(osp.abspath(output_filename)), suffix=".tar.gz.tmp")
    os.close(fd)
    try:
        with tarfile.open(temp_path, "w:gz") as tar:
            tar.add(source_dir, arcname=osp.basename(source_dir))
        # mkstemp creates the file as 0600; keep the umask-based mode a
        # plain open() would have given, so other users can still read it.
        umask = os.umask(0)
        os.umask(umask)
        os.chmod(temp_path, 0o666 & ~umask)
        os.replace(temp_path, output_filename)
    except BaseException:
        if osp.exists(temp_path):
            os.remove(temp_path)
        raise
    return output_filename


def is_valid_tarfile(filename):
    """Check that a tar archive can be read to the end.

    Parameters
    ----------
    filename : str
        Path to the archive.

    Returns
    -------
    bool
        False if the archive is truncated or otherwise unreadable.
    """
    try:
        with tarfile.open(filename, "r:*") as tar:
            for member in tar:
                if member.isfile():
                    f = tar.extractfile(member)
                    while f.read(1 << 20):
                        pass
    except (tarfile.TarError, EOFError, OSError):
        return False
    return True
