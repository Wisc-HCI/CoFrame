import { Tag, Input, Menu, Button, Dropdown} from 'antd';
import { DownOutlined, UserOutlined } from '@ant-design/icons';
import { Component } from 'react';

class EditableTagGroup extends Component {
  state = {
    tags: [],
    types: [],
    inputVisible: false,
    inputValue: '',
    inputType: '',
    editInputIndex: -1,
    editInputValue: '',
    editable: this.props.editable
  };

  handleClose = removedTag => {
    const ind = this.state.tags.indexOf(removedTag)
    if (ind > -1) {
      this.state.types.splice(ind, 1);
    }
    const tags = this.state.tags.filter(tag => tag !== removedTag);
    this.setState({ tags });
  };

  showInput = () => {
    this.setState({ inputVisible: true }, () => this.input.focus());
  };

  handleInputChange = e => {
    this.setState({ inputValue: e.target.value });
  };

  handleInputConfirm = () => {
    const { inputValue, inputType } = this.state;
    let { tags, types} = this.state;
    if (inputValue && tags.indexOf(inputValue) === -1) {
      tags = [...tags, inputValue];
      types = [...types, inputType];
    }
    this.setState({
      tags,
      types,
      inputVisible: false,
      inputValue: '',
      inputType: '',
    });
  };

  handleEditInputChange = e => {
    this.setState({ editInputValue: e.target.value });
  };

  handleEditInputConfirm = () => {
    this.setState(({ tags, editInputIndex, editInputValue }) => {
      const newTags = [...tags];
      newTags[editInputIndex] = editInputValue;

      return {
        tags: newTags,
        editInputIndex: -1,
        editInputValue: '',
      };
    });
  };

  saveInputRef = input => {
    this.input = input;
  };

  saveEditInputRef = input => {
    this.editInput = input;
  };
  
  handleMenuClick = e => {
    this.setState({
      inputType: e.key,
    });
    this.showInput();
  };

  render() {
    const { tags, inputVisible, inputValue, editInputIndex, editInputValue, editable } = this.state;
    const menu = (<Menu onClick={this.handleMenuClick}>
        <Menu.Item key="machine" icon={<UserOutlined />}>
          Machine
        </Menu.Item>
        <Menu.Item key="trajectory" icon={<UserOutlined />}>
          Tracjectory
        </Menu.Item>
        <Menu.Item key="location" icon={<UserOutlined />}>
          Location
        </Menu.Item>
        <Menu.Item key="thing" icon={<UserOutlined />}>
          Thing
        </Menu.Item>
      </Menu>);
    return (
      <>
        {tags.map((tag, index) => {
          if (editInputIndex === index) {
            return (
              <Input
                ref={this.saveEditInputRef}
                key={tag}
                size="small"
                className="tag-input"
                value={editInputValue}
                onChange={this.handleEditInputChange}
                onBlur={this.handleEditInputConfirm}
                onPressEnter={this.handleEditInputConfirm}
              />
            );
          }

          const isLongTag = tag.length > 20;

          const tagElem = (
            <Tag
              className="edit-tag"
              key={tag}
              closable={editable}
              onClose={() => this.handleClose(tag)}
            >
              <span
                onDoubleClick={e => {
                  if (editable) {
                    this.setState({ editInputIndex: index, editInputValue: tag }, () => {
                      this.editInput.focus();
                    });
                    e.preventDefault();
                  }
                }}
              >
                {isLongTag ? `${tag.slice(0, 20)}...` : tag}
              </span>
            </Tag>
          );
          return tagElem;
        })}
        {inputVisible && (
          <Input
            ref={this.saveInputRef}
            type="text"
            size="small"
            className="tag-input"
            value={inputValue}
            onChange={this.handleInputChange}
            onBlur={this.handleInputConfirm}
            onPressEnter={this.handleInputConfirm}
          />
        )}
        {editable && !inputVisible && (
          <div>
            <Dropdown overlay={menu}>
              <Button>
                New Parameter <DownOutlined />
              </Button>
            </Dropdown>
          </div>
        )}
      </>
    );
  }
}

export default EditableTagGroup;
