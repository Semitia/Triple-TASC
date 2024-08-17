import os
import h5py
import numpy as np


def pad_episode_len(group,org_len,full_len):
    for key in group:
        item = group[key]
        if isinstance(item, h5py.Dataset):
            name=item.name
            #print(f"{item.name} original shape= {item.shape}")
            last_row = item[-1, :]
            pad_rows = np.tile(last_row, (full_len-org_len, 1))  
            new_data = np.vstack((item[:], pad_rows))
            del file[item.name]  
            file.create_dataset(name, data=new_data)
            print(f"{name} after padding shape= {file[name].shape}")
        elif isinstance(item, h5py.Group):
            pad_episode_len(item,org_len,full_len)


hdf5_root="/home/oskar/hp09_ws/data/hdf5"
task_name="rotate"
episode_num=10


if __name__=="__main__":
    
    episode_lens=[]
    inhand_lens=[]
    overview_lens=[]
    for i in range(episode_num):
        file_name=f"episode_{i}.hdf5"
        file_path=os.path.join(hdf5_root,task_name,file_name)
        with h5py.File(file_path, 'r') as file:
            episode_lens.append(file['/action'].shape[0])
            inhand_lens.append(file['/observations/images/inhand'].shape[1])
            overview_lens.append(file['/observations/images/overview'].shape[1])
    print(f'episode_lens={episode_lens}')
    print(f'inhand_lens={inhand_lens}')
    print(f'overview_lens={overview_lens}')
    
    # pad episode len
    max_episode_len=max(episode_lens)
    for i in range(episode_num):
        file_name=f"episode_{i}.hdf5"
        file_path=os.path.join(hdf5_root,task_name,file_name)
        with h5py.File(file_path, 'r+') as file:
            print('\n')
            print(f"file: episode_{i}")
            org_len=file['/action'].shape[0]
            pad_episode_len(file,org_len,max_episode_len)

    # pad inhand and overview images
    max_inhand_len=max(inhand_lens)
    max_overview_len=max(overview_lens)
    for i in range(episode_num):
        file_name=f"episode_{i}.hdf5"
        file_path=os.path.join(hdf5_root,task_name,file_name)
        with h5py.File(file_path, 'r+') as file:
            print('\n')
            print(f"file: episode_{i}")

            org_inhand_len=file['/observations/images/inhand'].shape[1]
            padded_inhand_data=np.pad(file['/observations/images/inhand'], ((0, 0), (0, max_inhand_len-org_inhand_len)), mode='constant', constant_values=0)
            del file['/observations/images/inhand']
            file.create_dataset('/observations/images/inhand',data=padded_inhand_data)
            print(f'after padding inhand_shape={file["/observations/images/inhand"].shape}')

            org_overview_len=file['/observations/images/overview'].shape[1]
            padded_overview_data=np.pad(file['/observations/images/overview'], ((0, 0), (0, max_overview_len-org_overview_len)), mode='constant', constant_values=0)
            del file['/observations/images/overview']
            file.create_dataset('/observations/images/overview',data=padded_overview_data)
            print(f'after padding overview_shape={file["/observations/images/overview"].shape}')



